#include "realtime_loop.h"

#include "driver/gptimer.h"
#include "driver/i2c_master.h"
#include "engine_angle_kalman.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "kalman.h"
#include "motor_controller.h"
#include "mt6701.h"
#include <inttypes.h>
#include <limits.h>

#define CONTROL_CORE_ID 1
#define HOUSEKEEPING_CORE_ID 0
#define I2C_PORT_NUM I2C_NUM_0
#define I2C_CLOCK_HZ 400000U
#define TIMER_RESOLUTION_HZ 1000000U
#define SENSOR_PERIOD_US (TIMER_RESOLUTION_HZ / REALTIME_SENSOR_RATE_HZ)
#define CONTROL_DIVIDER (REALTIME_SENSOR_RATE_HZ / REALTIME_CONTROL_RATE_HZ)
#define TELEMETRY_PERIOD_US 5000000LL
#define DUMMY_MOTOR_OUTPUT_PERCENT 50.0f
#define KALMAN_REFERENCE_RATE_HZ 1000.0f
#define DEGREES_PER_SECOND_TO_RPM (1.0f / 6.0f)
#define MICROSECONDS_TO_SECONDS (1.0f / 1000000.0f)
#define REALTIME_TASK_STACK_SIZE 4096U
#define LOGGER_TASK_STACK_SIZE 4096U
#define REALTIME_TASK_PRIORITY (configMAX_PRIORITIES - 1)
#define LOGGER_TASK_PRIORITY 1

#if CONFIG_FREERTOS_NUMBER_OF_CORES < 2
#error "The deterministic control loop requires both ESP32-S3 CPU cores"
#endif

_Static_assert(TIMER_RESOLUTION_HZ % REALTIME_SENSOR_RATE_HZ == 0,
               "Sensor period must be an integer number of timer ticks");
_Static_assert(REALTIME_SENSOR_RATE_HZ % REALTIME_CONTROL_RATE_HZ == 0,
               "Sensor rate must be divisible by control rate");

static const char *TAG = "REALTIME_LOOP";

typedef struct {
  struct engine_config *motor;
  i2c_master_bus_handle_t i2c_bus;
  i2c_master_dev_handle_t i2c_device;
  mt6701_dev_t sensor;
  struct kalman_3d filter;
} realtime_loop_context_t;

typedef struct {
  uint32_t estimator_updates;
  uint32_t control_updates;
  uint32_t missed_timer_events;
  uint32_t sensor_errors;
  uint32_t deadline_overruns;
  uint32_t window_duration_us;
  uint32_t max_wake_latency_us;
  uint32_t max_i2c_time_us;
  uint32_t max_kalman_time_us;
  uint32_t max_control_time_us;
  uint32_t max_processing_time_us;
  uint32_t max_cycle_time_us;
  uint32_t lifetime_max_processing_time_us;
  uint32_t previous_telemetry_time_us;
  uint32_t min_sample_dt_us;
  uint32_t max_sample_dt_us;
  uint32_t min_control_dt_us;
  uint32_t max_control_dt_us;
  uint32_t total_estimator_updates;
  uint32_t total_control_updates;
  uint32_t total_missed_timer_events;
  uint32_t total_sensor_errors;
  int32_t total_turns;
  float measured_angle_deg;
  float estimated_angle_deg;
  float estimated_speed_rpm;
  float estimated_acceleration_rpm_s;
  float motor_output_percent;
} realtime_telemetry_t;

typedef struct {
  int64_t start_time_us;
  uint32_t estimator_updates;
  uint32_t control_updates;
  uint32_t missed_timer_events;
  uint32_t sensor_errors;
  uint32_t deadline_overruns;
  uint32_t max_wake_latency_us;
  uint32_t max_i2c_time_us;
  uint32_t max_kalman_time_us;
  uint32_t max_control_time_us;
  uint32_t max_processing_time_us;
  uint32_t max_cycle_time_us;
  uint32_t min_sample_dt_us;
  uint32_t max_sample_dt_us;
  uint32_t min_control_dt_us;
  uint32_t max_control_dt_us;
} timing_window_t;

static realtime_loop_context_t loop_context;
static QueueHandle_t telemetry_queue;
static volatile uint32_t last_timer_isr_time_us;

static inline void update_max(uint32_t *maximum, uint32_t value) {
  if (value > *maximum) {
    *maximum = value;
  }
}

static inline void update_range(uint32_t *minimum, uint32_t *maximum,
                                uint32_t value) {
  if (value < *minimum) {
    *minimum = value;
  }
  update_max(maximum, value);
}

static void reset_timing_window(timing_window_t *window,
                                int64_t start_time_us) {
  *window = (timing_window_t){
      .start_time_us = start_time_us,
      .min_sample_dt_us = UINT32_MAX,
      .min_control_dt_us = UINT32_MAX,
  };
}

static uint32_t valid_minimum(uint32_t minimum) {
  return minimum == UINT32_MAX ? 0 : minimum;
}

static bool IRAM_ATTR sampling_timer_callback(
    gptimer_handle_t timer, const gptimer_alarm_event_data_t *event_data,
    void *user_context) {
  (void)timer;
  (void)event_data;

  last_timer_isr_time_us = (uint32_t)esp_timer_get_time();
  BaseType_t high_priority_task_woken = pdFALSE;
  vTaskNotifyGiveFromISR((TaskHandle_t)user_context, &high_priority_task_woken);
  return high_priority_task_woken == pdTRUE;
}

static esp_err_t initialize_sensor_and_filter(realtime_loop_context_t *context,
                                              float *initial_angle_deg) {
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
      .scl_speed_hz = I2C_CLOCK_HZ,
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

static void logger_task(void *argument) {
  (void)argument;
  realtime_telemetry_t telemetry;
  bool discard_contaminated_window = false;

  ESP_LOGI(TAG, "Telemetry logger running on Core %d", xPortGetCoreID());
  while (true) {
    if (xQueueReceive(telemetry_queue, &telemetry, portMAX_DELAY) != pdTRUE) {
      continue;
    }

    /*
     * Printing the previous report can disturb the following 5-second timing
     * window through shared SoC resources. Discard that contaminated window
     * without logging, then report the next fully silent window. This produces
     * one trustworthy timing report every 10 seconds.
     */
    if (discard_contaminated_window) {
      discard_contaminated_window = false;
      continue;
    }
    discard_contaminated_window = true;

    float window_seconds =
        (float)telemetry.window_duration_us * MICROSECONDS_TO_SECONDS;
    float estimator_rate_hz =
        (float)telemetry.estimator_updates / window_seconds;
    float control_rate_hz = (float)telemetry.control_updates / window_seconds;

    ESP_LOGI(TAG,
             "silent window: angle=%.3f/%.3f deg speed=%.3f RPM "
             "accel=%.3f RPM/s "
             "turns=%" PRId32 " output=%.1f%%",
             telemetry.measured_angle_deg, telemetry.estimated_angle_deg,
             telemetry.estimated_speed_rpm,
             telemetry.estimated_acceleration_rpm_s, telemetry.total_turns,
             telemetry.motor_output_percent);
    ESP_LOGI(TAG,
             "window: rate=%.1f/%.1f Hz missed=%" PRIu32 " errors=%" PRIu32
             " overruns=%" PRIu32,
             estimator_rate_hz, control_rate_hz, telemetry.missed_timer_events,
             telemetry.sensor_errors, telemetry.deadline_overruns);
    ESP_LOGI(TAG,
             "window max: wake=%" PRIu32 " i2c=%" PRIu32 " kalman=%" PRIu32
             " control=%" PRIu32 " processing=%" PRIu32 " cycle=%" PRIu32
             " us lifetime_processing=%" PRIu32
             " us previous_telemetry=%" PRIu32 " us",
             telemetry.max_wake_latency_us, telemetry.max_i2c_time_us,
             telemetry.max_kalman_time_us, telemetry.max_control_time_us,
             telemetry.max_processing_time_us, telemetry.max_cycle_time_us,
             telemetry.lifetime_max_processing_time_us,
             telemetry.previous_telemetry_time_us);
    ESP_LOGI(TAG,
             "window dt: sample=%" PRIu32 "..%" PRIu32 " us control=%" PRIu32
             "..%" PRIu32 " us totals: est=%" PRIu32 " ctl=%" PRIu32
             " missed=%" PRIu32 " errors=%" PRIu32,
             telemetry.min_sample_dt_us, telemetry.max_sample_dt_us,
             telemetry.min_control_dt_us, telemetry.max_control_dt_us,
             telemetry.total_estimator_updates, telemetry.total_control_updates,
             telemetry.total_missed_timer_events,
             telemetry.total_sensor_errors);
  }
}

static uint32_t publish_telemetry(
    realtime_loop_context_t *context, timing_window_t *window,
    int64_t publish_time_us, uint32_t lifetime_max_processing_time_us,
    uint32_t previous_telemetry_time_us, uint32_t total_estimator_updates,
    uint32_t total_control_updates, uint32_t total_missed_timer_events,
    uint32_t total_sensor_errors, float measured_angle_deg,
    float motor_output_percent) {
  int64_t telemetry_start_us = esp_timer_get_time();
  int32_t total_turns = 0;
  mt6701_get_total_turns(&context->sensor, &total_turns);

  realtime_telemetry_t telemetry = {
      .estimator_updates = window->estimator_updates,
      .control_updates = window->control_updates,
      .missed_timer_events = window->missed_timer_events,
      .sensor_errors = window->sensor_errors,
      .deadline_overruns = window->deadline_overruns,
      .window_duration_us = (uint32_t)(publish_time_us - window->start_time_us),
      .max_wake_latency_us = window->max_wake_latency_us,
      .max_i2c_time_us = window->max_i2c_time_us,
      .max_kalman_time_us = window->max_kalman_time_us,
      .max_control_time_us = window->max_control_time_us,
      .max_processing_time_us = window->max_processing_time_us,
      .max_cycle_time_us = window->max_cycle_time_us,
      .lifetime_max_processing_time_us = lifetime_max_processing_time_us,
      .previous_telemetry_time_us = previous_telemetry_time_us,
      .min_sample_dt_us = valid_minimum(window->min_sample_dt_us),
      .max_sample_dt_us = window->max_sample_dt_us,
      .min_control_dt_us = valid_minimum(window->min_control_dt_us),
      .max_control_dt_us = window->max_control_dt_us,
      .total_estimator_updates = total_estimator_updates,
      .total_control_updates = total_control_updates,
      .total_missed_timer_events = total_missed_timer_events,
      .total_sensor_errors = total_sensor_errors,
      .total_turns = total_turns,
      .measured_angle_deg = measured_angle_deg,
      .estimated_angle_deg = context->filter.x[0],
      .estimated_speed_rpm = context->filter.x[1] * DEGREES_PER_SECOND_TO_RPM,
      .estimated_acceleration_rpm_s =
          context->filter.x[2] * DEGREES_PER_SECOND_TO_RPM,
      .motor_output_percent = motor_output_percent,
  };
  xQueueOverwrite(telemetry_queue, &telemetry);
  reset_timing_window(window, publish_time_us);
  return (uint32_t)(esp_timer_get_time() - telemetry_start_us);
}

static void realtime_task(void *argument) {
  realtime_loop_context_t *context = (realtime_loop_context_t *)argument;
  motor_controller_t controller;
  motor_controller_init(&controller, DUMMY_MOTOR_OUTPUT_PERCENT);

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
           "I2C=%u Hz initial=%.3f deg dummy=%.1f%%",
           xPortGetCoreID(), REALTIME_SENSOR_RATE_HZ, REALTIME_CONTROL_RATE_HZ,
           I2C_CLOCK_HZ, measured_angle_deg, DUMMY_MOTOR_OUTPUT_PERCENT);

  gptimer_handle_t sampling_timer = NULL;
  err = start_sampling_timer(xTaskGetCurrentTaskHandle(), &sampling_timer);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Sampling timer initialization failed: %s",
             esp_err_to_name(err));
    engine_driver_set_speed(context->motor, 0.0f);
    vTaskDelete(NULL);
    return;
  }

  uint32_t control_divider = 0;
  uint32_t total_estimator_updates = 0;
  uint32_t total_control_updates = 0;
  uint32_t total_missed_timer_events = 0;
  uint32_t total_sensor_errors = 0;
  uint32_t lifetime_max_processing_time_us = 0;
  uint32_t previous_telemetry_time_us = 0;
  int64_t last_sample_time_us = esp_timer_get_time();
  int64_t last_control_time_us = last_sample_time_us;
  float motor_output_percent = 0.0f;
  timing_window_t window;
  reset_timing_window(&window, last_sample_time_us);

  while (true) {
    uint32_t pending_events = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    uint32_t wake_time_us = (uint32_t)esp_timer_get_time();
    uint32_t wake_latency_us = wake_time_us - last_timer_isr_time_us;
    update_max(&window.max_wake_latency_us, wake_latency_us);

    if (pending_events > 1) {
      uint32_t missed = pending_events - 1;
      window.missed_timer_events += missed;
      total_missed_timer_events += missed;
    }

    int64_t processing_start_us = esp_timer_get_time();
    int64_t i2c_start_us = processing_start_us;
    err = mt6701_update(&context->sensor);
    int64_t i2c_end_us = esp_timer_get_time();
    update_max(&window.max_i2c_time_us, (uint32_t)(i2c_end_us - i2c_start_us));

    if (err == ESP_OK) {
      uint32_t sample_dt_us = (uint32_t)(i2c_end_us - last_sample_time_us);
      last_sample_time_us = i2c_end_us;
      update_range(&window.min_sample_dt_us, &window.max_sample_dt_us,
                   sample_dt_us);

      int64_t kalman_start_us = esp_timer_get_time();
      float sample_dt = (float)sample_dt_us * MICROSECONDS_TO_SECONDS;
      if (mt6701_get_last_angle_degrees(&context->sensor,
                                        &measured_angle_deg) == ESP_OK &&
          sample_dt > 0.0f && sample_dt < 0.1f) {
        engine_angle_kalman_3d_update(&context->filter, measured_angle_deg,
                                      sample_dt);
        window.estimator_updates++;
        total_estimator_updates++;
      }
      update_max(&window.max_kalman_time_us,
                 (uint32_t)(esp_timer_get_time() - kalman_start_us));
    } else {
      window.sensor_errors++;
      total_sensor_errors++;
    }

    control_divider++;
    if (control_divider >= CONTROL_DIVIDER) {
      control_divider = 0;
      int64_t control_start_us = esp_timer_get_time();
      uint32_t control_dt_us =
          (uint32_t)(control_start_us - last_control_time_us);
      last_control_time_us = control_start_us;
      update_range(&window.min_control_dt_us, &window.max_control_dt_us,
                   control_dt_us);

      float estimated_speed_rpm =
          context->filter.x[1] * DEGREES_PER_SECOND_TO_RPM;
      motor_output_percent = motor_controller_update(
          &controller, estimated_speed_rpm,
          (float)control_dt_us * MICROSECONDS_TO_SECONDS);
      engine_driver_set_speed(context->motor, motor_output_percent);
      window.control_updates++;
      total_control_updates++;
      update_max(&window.max_control_time_us,
                 (uint32_t)(esp_timer_get_time() - control_start_us));
    }

    int64_t processing_end_us = esp_timer_get_time();
    uint32_t processing_time_us =
        (uint32_t)(processing_end_us - processing_start_us);
    uint32_t cycle_time_us = wake_latency_us + processing_time_us;
    update_max(&window.max_processing_time_us, processing_time_us);
    update_max(&window.max_cycle_time_us, cycle_time_us);
    update_max(&lifetime_max_processing_time_us, processing_time_us);
    if (cycle_time_us > SENSOR_PERIOD_US) {
      window.deadline_overruns++;
    }

    if (processing_end_us - window.start_time_us >= TELEMETRY_PERIOD_US) {
      previous_telemetry_time_us = publish_telemetry(
          context, &window, processing_end_us, lifetime_max_processing_time_us,
          previous_telemetry_time_us, total_estimator_updates,
          total_control_updates, total_missed_timer_events, total_sensor_errors,
          measured_angle_deg, motor_output_percent);

      uint32_t processing_with_telemetry_us =
          (uint32_t)(esp_timer_get_time() - processing_start_us);
      update_max(&lifetime_max_processing_time_us,
                 processing_with_telemetry_us);
      update_max(&window.max_processing_time_us, processing_with_telemetry_us);
      update_max(&window.max_cycle_time_us,
                 wake_latency_us + processing_with_telemetry_us);
      if (cycle_time_us <= SENSOR_PERIOD_US &&
          wake_latency_us + processing_with_telemetry_us > SENSOR_PERIOD_US) {
        window.deadline_overruns++;
      }
    }
  }
}

esp_err_t realtime_loop_start(struct engine_config *motor) {
  if (motor == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  telemetry_queue = xQueueCreate(1, sizeof(realtime_telemetry_t));
  if (telemetry_queue == NULL) {
    return ESP_ERR_NO_MEM;
  }

  loop_context.motor = motor;

  TaskHandle_t logger_handle = NULL;
  BaseType_t task_created = xTaskCreatePinnedToCore(
      logger_task, "telemetry_logger", LOGGER_TASK_STACK_SIZE, NULL,
      LOGGER_TASK_PRIORITY, &logger_handle, HOUSEKEEPING_CORE_ID);
  if (task_created != pdPASS) {
    vQueueDelete(telemetry_queue);
    telemetry_queue = NULL;
    return ESP_ERR_NO_MEM;
  }

  task_created = xTaskCreatePinnedToCore(
      realtime_task, "motor_realtime", REALTIME_TASK_STACK_SIZE, &loop_context,
      REALTIME_TASK_PRIORITY, NULL, CONTROL_CORE_ID);
  if (task_created != pdPASS) {
    vTaskDelete(logger_handle);
    vQueueDelete(telemetry_queue);
    telemetry_queue = NULL;
    return ESP_ERR_NO_MEM;
  }

  return ESP_OK;
}
