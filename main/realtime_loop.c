/*
 * Dual-rate motor runtime for ESP32-S3.
 *
 * Core 1 owns the complete deterministic path:
 *   GPTimer (4 kHz) -> MT6701 -> Kalman -> controller (1 kHz) -> MCPWM.
 *
 * Core 0 owns presentation only:
 *   telemetry queue -> formatted ESP_LOGI output.
 *
 * Timing instrumentation runs on Core 1 because it measures the critical path,
 * but it never prints. Telemetry copies those measurements to Core 0 and never
 * participates in the control law. See docs/arquitetura-tempo-real.md.
 */
#include "realtime_loop.h"

#include "driver/gptimer.h"
#include "driver/i2c_master.h"
#include "engine_angle_kalman.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_rt_diagnostics.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "kalman.h"
#include "motor_controller.h"
#include "mt6701.h"
#include "realtime_telemetry.h"

/* Core allocation: keep console/system work away from the control core. */
#define CONTROL_CORE_ID 1

/* Hardware scheduling and communication. One timer tick equals one us. */
#define I2C_PORT_NUM I2C_NUM_0
#define TIMER_RESOLUTION_HZ 1000000U
#define SENSOR_PERIOD_US (TIMER_RESOLUTION_HZ / REALTIME_SENSOR_RATE_HZ)
#define CONTROL_DIVIDER (REALTIME_SENSOR_RATE_HZ / REALTIME_CONTROL_RATE_HZ)

/* Filter tuning reference and unit conversions. */
#define KALMAN_REFERENCE_RATE_HZ 1000.0f
#define DEGREES_PER_SECOND_TO_RPM (1.0f / 6.0f)
#define MICROSECONDS_TO_SECONDS (1.0f / 1000000.0f)

/* FreeRTOS task resources. */
#define REALTIME_TASK_STACK_SIZE 4096U
#define REALTIME_TASK_PRIORITY (configMAX_PRIORITIES - 1)

#if CONFIG_FREERTOS_NUMBER_OF_CORES < 2
#error "The deterministic control loop requires both ESP32-S3 CPU cores"
#endif

_Static_assert(TIMER_RESOLUTION_HZ % REALTIME_SENSOR_RATE_HZ == 0,
               "Sensor period must be an integer number of timer ticks");
_Static_assert(REALTIME_SENSOR_RATE_HZ % REALTIME_CONTROL_RATE_HZ == 0,
               "Sensor rate must be divisible by control rate");
_Static_assert(REALTIME_DIAG_STAGE_COUNT <= ESP_RT_DIAG_MAX_STAGES,
               "Application declares too many diagnostic stages");
_Static_assert(REALTIME_DIAG_EVENT_COUNT <= ESP_RT_DIAG_MAX_EVENTS,
               "Application declares too many diagnostic events");
_Static_assert(REALTIME_DIAG_INTERVAL_COUNT <= ESP_RT_DIAG_MAX_INTERVALS,
               "Application declares too many diagnostic intervals");

static const char *TAG = "REALTIME_LOOP";

/* Functional state owned exclusively by the Core 1 real-time task. */
typedef struct {
  struct engine_config *motor;        /* Initialized MCPWM driver state. */
  i2c_master_bus_handle_t i2c_bus;    /* ESP-IDF I2C controller handle. */
  i2c_master_dev_handle_t i2c_device; /* MT6701 at address 0x06. */
  mt6701_dev_t sensor;                /* Sensor cache and turn tracking. */
  struct kalman_3d filter;            /* [angle, speed, acceleration]. */
} realtime_loop_context_t;

/* Static lifetime is required because app_main returns after creating tasks. */
static realtime_loop_context_t loop_context;

/* Written by the GPTimer ISR and read once by the awakened Core 1 task. */
static volatile uint32_t last_timer_isr_time_us;

/* ============================ 4 KHZ SCHEDULER ============================ */

/*
 * GPTimer ISR. Keep this callback bounded and IRAM-safe: timestamp the event,
 * notify the already-created task, and request an immediate context switch.
 * Sensor access and all floating-point work deliberately remain outside ISR.
 */
static bool IRAM_ATTR sampling_timer_callback(
    gptimer_handle_t timer, const gptimer_alarm_event_data_t *event_data,
    void *user_context) {
  (void)timer;
  (void)event_data;

  esp_rt_diag_isr_capture(&last_timer_isr_time_us);
  BaseType_t high_priority_task_woken = pdFALSE;
  vTaskNotifyGiveFromISR((TaskHandle_t)user_context, &high_priority_task_woken);
  return high_priority_task_woken == pdTRUE;
}

/*
 * Create the I2C bus and MT6701 from Core 1, obtain the first valid angle, then
 * initialize the Kalman state at that angle. Starting from the current angle
 * avoids a large artificial innovation during the first filter update.
 */
static esp_err_t initialize_sensor_and_filter(realtime_loop_context_t *context,
                                              float *initial_angle_deg) {
  /*
   * Internal pull-ups are enabled as a fallback. At the configured 1 MHz,
   * suitable external pull-ups and short wiring are still required to meet the
   * MT6701 rise/fall-time specification.
   */
  i2c_master_bus_config_t bus_config = {
      .i2c_port = I2C_PORT_NUM,
      .sda_io_num = CONFIG_APP_I2C_SDA_PIN,
      .scl_io_num = CONFIG_APP_I2C_SCL_PIN,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 7,
      .flags.enable_internal_pullup = true,
  };
  ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_config, &context->i2c_bus), TAG,
                      "Could not create I2C bus");

  i2c_device_config_t device_config = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = MT6701_I2C_ADDRESS,
      .scl_speed_hz = CONFIG_APP_I2C_CLOCK_HZ,
  };
  ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(context->i2c_bus,
                                                &device_config,
                                                &context->i2c_device),
                      TAG, "Could not add MT6701 to I2C bus");
  ESP_RETURN_ON_ERROR(mt6701_init(&context->sensor, context->i2c_device), TAG,
                      "Could not initialize MT6701");
  ESP_RETURN_ON_ERROR(
      mt6701_set_software_direction(&context->sensor, MT6701_DIR_CW), TAG,
      "Could not configure MT6701 direction");
  ESP_RETURN_ON_ERROR(
      mt6701_get_last_angle_degrees(&context->sensor, initial_angle_deg), TAG,
      "Could not get initial MT6701 angle");

  /*
   * Q values were originally tuned for a 1 kHz update. Scale their per-update
   * contribution by 1/4 because this estimator runs four times faster. R is
   * the angle-measurement variance and therefore is not rate-scaled here.
   */
  const float q_rate_scale =
      KALMAN_REFERENCE_RATE_HZ / (float)REALTIME_SENSOR_RATE_HZ;
  kalman_3d_config_t filter_config = {
      .q_theta = 0.001f * q_rate_scale,
      .q_omega = 10.0f * q_rate_scale,
      .q_alpha = 100.0f * q_rate_scale,
      .r = 0.0004f,
  };
  kalman_3d_init(&context->filter, *initial_angle_deg, &filter_config);
  return ESP_OK;
}

/*
 * Configure an auto-reloading 250 us GPTimer. user_context is the task handle
 * that the ISR will notify on every alarm.
 */
static esp_err_t start_sampling_timer(TaskHandle_t realtime_task,
                                      gptimer_handle_t *timer) {
  gptimer_config_t timer_config = {
      .clk_src = GPTIMER_CLK_SRC_DEFAULT,
      .direction = GPTIMER_COUNT_UP,
      .resolution_hz = TIMER_RESOLUTION_HZ,
  };
  ESP_RETURN_ON_ERROR(gptimer_new_timer(&timer_config, timer), TAG,
                      "Could not create GPTimer");

  gptimer_event_callbacks_t callbacks = {
      .on_alarm = sampling_timer_callback,
  };
  ESP_RETURN_ON_ERROR(
      gptimer_register_event_callbacks(*timer, &callbacks, realtime_task), TAG,
      "Could not register GPTimer callback");

  gptimer_alarm_config_t alarm_config = {
      .alarm_count = SENSOR_PERIOD_US,
      .reload_count = 0,
      .flags.auto_reload_on_alarm = true,
  };
  ESP_RETURN_ON_ERROR(gptimer_set_alarm_action(*timer, &alarm_config), TAG,
                      "Could not configure GPTimer alarm");
  ESP_RETURN_ON_ERROR(gptimer_enable(*timer), TAG, "Could not enable GPTimer");
  return gptimer_start(*timer);
}

/*
 * Assemble one coherent snapshot from live Core 1 state. The telemetry module
 * owns the queue and all Core 0 formatting; this function only maps loop state
 * into the transport structure and resets the instrumentation window.
 *
 */
static void publish_telemetry_snapshot(realtime_loop_context_t *context,
                                       esp_rt_diag_t *diagnostics,
                                       int64_t publish_time_us,
                                       float measured_angle_deg,
                                       const motor_controller_t *controller) {
  int64_t snapshot_start_us = esp_rt_diag_stage_begin();
  esp_rt_diag_snapshot_t diagnostics_snapshot;
  if (esp_rt_diag_take_snapshot(diagnostics, publish_time_us,
                                &diagnostics_snapshot) != ESP_OK) {
    return;
  }

  /* Turn count comes from MT6701 tracking; speed comes from Kalman below. */
  int32_t total_turns = 0;
  mt6701_get_total_turns(&context->sensor, &total_turns);
  motor_controller_status_t controller_status = {0};
  motor_controller_get_status(controller, &controller_status);

  realtime_telemetry_snapshot_t telemetry = {
      .diagnostics = diagnostics_snapshot,
      .total_turns = total_turns,
      .measured_angle_deg = measured_angle_deg,
      .estimated_angle_deg = context->filter.x[0],
      .estimated_speed_rpm = context->filter.x[1] * DEGREES_PER_SECOND_TO_RPM,
      .estimated_acceleration_rpm_s =
          context->filter.x[2] * DEGREES_PER_SECOND_TO_RPM,
      .speed_reference_rpm = controller_status.reference_rpm,
      .speed_error_rpm = controller_status.error_rpm,
      .pid_proportional_term = controller_status.proportional_term,
      .pid_integral_term = controller_status.integral_term,
      .pid_derivative_term = controller_status.derivative_term,
      .motor_output_percent = controller_status.output_percent,
  };
  realtime_telemetry_publish(&telemetry);
  esp_rt_diag_stage_end(diagnostics, REALTIME_DIAG_STAGE_SNAPSHOT,
                        snapshot_start_us);
}

/* ============= CORE 1 ACQUISITION, ESTIMATION AND CONTROL ============== */

/*
 * Highest-priority application task, permanently pinned to Core 1.
 *
 * Fast path on every timer event (4 kHz): sensor -> measured dt -> Kalman.
 * Divided path on every fourth event (1 kHz): controller -> motor driver.
 * Side path at the configured window: copy diagnostics to the Core 0 logger.
 */
static void realtime_task(void *argument) {
  realtime_loop_context_t *context = (realtime_loop_context_t *)argument;
  motor_controller_t controller;
  motor_controller_init(&controller);

  float measured_angle_deg = 0.0f;
  esp_err_t err = initialize_sensor_and_filter(context, &measured_angle_deg);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Real-time initialization failed: %s", esp_err_to_name(err));
    engine_driver_set_speed(context->motor, 0.0f);
    vTaskDelete(NULL);
    return;
  }

  ESP_LOGI(TAG,
           "Control on Core %d: sensor/Kalman=%u Hz control=%u Hz "
           "I2C=%u Hz initial=%.3f deg PID closed-loop",
           xPortGetCoreID(), REALTIME_SENSOR_RATE_HZ, REALTIME_CONTROL_RATE_HZ,
           CONFIG_APP_I2C_CLOCK_HZ, measured_angle_deg);

  gptimer_handle_t sampling_timer = NULL;
  err = start_sampling_timer(xTaskGetCurrentTaskHandle(), &sampling_timer);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Sampling timer initialization failed: %s",
             esp_err_to_name(err));
    engine_driver_set_speed(context->motor, 0.0f);
    vTaskDelete(NULL);
    return;
  }

  /* Divider and timestamps persist across diagnostic window resets. */
  uint32_t control_divider = 0;
  /* dt timestamps use actual completion/start instants rather than nominal dt.
   */
  int64_t last_sample_time_us = esp_timer_get_time();
  int64_t last_control_time_us = last_sample_time_us;
  float motor_output_percent = 0.0f;
#if CONFIG_ESP_RT_DIAGNOSTICS_ENABLE
  esp_rt_diag_t diagnostics;
  esp_rt_diag_t *diagnostics_ptr = &diagnostics;
  const esp_rt_diag_config_t diagnostics_config = {
      .expected_period_us = SENSOR_PERIOD_US,
      .deadline_us = SENSOR_PERIOD_US,
      .stage_count = REALTIME_DIAG_STAGE_COUNT,
      .event_count = REALTIME_DIAG_EVENT_COUNT,
      .interval_count = REALTIME_DIAG_INTERVAL_COUNT,
  };
  err =
      esp_rt_diag_init(&diagnostics, &diagnostics_config, last_sample_time_us);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Could not initialize real-time diagnostics: %s",
             esp_err_to_name(err));
    engine_driver_set_speed(context->motor, 0.0f);
    vTaskDelete(NULL);
    return;
  }
#else
  esp_rt_diag_t *const diagnostics_ptr = NULL;
#endif

  while (true) {
    /* Scheduler: wait for the 4 kHz GPTimer notification. */
    uint32_t pending_events = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    esp_rt_diag_cycle_begin_from_isr(diagnostics_ptr, &last_timer_isr_time_us,
                                     pending_events);

    /* Acquisition: one MT6701 burst read and cached-state update. */
    int64_t i2c_start_us = esp_rt_diag_stage_begin();
    err = mt6701_update(&context->sensor);
    int64_t i2c_end_us = esp_timer_get_time();
    esp_rt_diag_stage_record(diagnostics_ptr, REALTIME_DIAG_STAGE_I2C,
                             i2c_start_us, i2c_end_us);

    if (err == ESP_OK) {
      /* Timestamp the sample when the I2C transaction has completed. */
      uint32_t sample_dt_us = (uint32_t)(i2c_end_us - last_sample_time_us);
      last_sample_time_us = i2c_end_us;
      esp_rt_diag_interval(diagnostics_ptr, REALTIME_DIAG_INTERVAL_SAMPLE,
                           sample_dt_us);

      /* Estimation: update angle, speed and acceleration at 4 kHz. */
      int64_t kalman_start_us = esp_rt_diag_stage_begin();
      float sample_dt = (float)sample_dt_us * MICROSECONDS_TO_SECONDS;
      /*
       * get_last reads the value cached by mt6701_update(); it causes no second
       * I2C transaction. Reject pathological dt values after a long disruption.
       */
      if (mt6701_get_last_angle_degrees(&context->sensor,
                                        &measured_angle_deg) == ESP_OK &&
          sample_dt > 0.0f && sample_dt < 0.1f) {
        engine_angle_kalman_3d_update(&context->filter, measured_angle_deg,
                                      sample_dt);
        esp_rt_diag_event(diagnostics_ptr, REALTIME_DIAG_EVENT_ESTIMATOR_UPDATE,
                          1U);
      }
      esp_rt_diag_stage_end(diagnostics_ptr, REALTIME_DIAG_STAGE_KALMAN,
                            kalman_start_us);
    } else {
      /* Preserve the previous Kalman state and retry the sensor next period. */
      esp_rt_diag_event(diagnostics_ptr, REALTIME_DIAG_EVENT_SENSOR_ERROR, 1U);
    }

    /* Control: every fourth estimator cycle produces the 1 kHz motor action. */
    control_divider++;
    if (control_divider >= CONTROL_DIVIDER) {
      control_divider = 0;
      int64_t control_start_us = esp_timer_get_time();
      uint32_t control_dt_us =
          (uint32_t)(control_start_us - last_control_time_us);
      last_control_time_us = control_start_us;
      esp_rt_diag_interval(diagnostics_ptr, REALTIME_DIAG_INTERVAL_CONTROL,
                           control_dt_us);
      int64_t control_stage_start_us = esp_rt_diag_stage_begin();

      /* Kalman x[1] is deg/s; 360 deg/rev and 60 s/min give 1 RPM per 6 deg/s.
       */
      float estimated_speed_rpm =
          context->filter.x[1] * DEGREES_PER_SECOND_TO_RPM;
      motor_output_percent = motor_controller_update(
          &controller, estimated_speed_rpm,
          (float)control_dt_us * MICROSECONDS_TO_SECONDS);
      /* Apply the saturated 0..100% command returned by the PID controller. */
      engine_driver_set_speed(context->motor, motor_output_percent);
      esp_rt_diag_event(diagnostics_ptr, REALTIME_DIAG_EVENT_CONTROL_UPDATE,
                        1U);
      esp_rt_diag_stage_end(diagnostics_ptr, REALTIME_DIAG_STAGE_CONTROL,
                            control_stage_start_us);
    }

    /* Instrumentation: measure this cycle without printing from Core 1. */
    int64_t processing_end_us = esp_rt_diag_cycle_end_now(diagnostics_ptr);

    /* Snapshot and application payload are copied only when the window ends. */
    if (esp_rt_diag_snapshot_due(diagnostics_ptr, processing_end_us)) {
      publish_telemetry_snapshot(context, diagnostics_ptr, processing_end_us,
                                 measured_angle_deg, &controller);
    }
  }
}

/*
 * Public startup entry point. Create telemetry first and real-time second so
 * every published snapshot has a consumer. All objects have application
 * lifetime after successful startup.
 */
esp_err_t realtime_loop_start(struct engine_config *motor) {
  if (motor == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  loop_context.motor = motor;

#if CONFIG_ESP_RT_DIAGNOSTICS_ENABLE
  esp_err_t err = realtime_telemetry_start();
  if (err != ESP_OK) {
    return err;
  }
#endif

  /* Sensor, estimator and controller share Core 1 and the highest app priority.
   */
  BaseType_t task_created = xTaskCreatePinnedToCore(
      realtime_task, "motor_realtime", REALTIME_TASK_STACK_SIZE, &loop_context,
      REALTIME_TASK_PRIORITY, NULL, CONTROL_CORE_ID);
  if (task_created != pdPASS) {
#if CONFIG_ESP_RT_DIAGNOSTICS_ENABLE
    realtime_telemetry_stop();
#endif
    return ESP_ERR_NO_MEM;
  }

  return ESP_OK;
}
