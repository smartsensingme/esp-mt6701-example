/*
 * Dual-rate motor runtime for ESP32-S3.
 *
 * Core 1 owns the complete deterministic path:
 *   GPTimer (3 kHz average) -> synchronous MT6701 -> Kalman
 *   -> controller (1 kHz)
 *   -> MCPWM.
 *
 * Core 0 owns presentation only:
 *   diagnostic reporter -> generic and application ESP_LOGI output.
 *
 * Timing instrumentation runs on Core 1 because it measures the critical path,
 * but it never prints. Telemetry copies those measurements to Core 0 and never
 * participates in the control law. See docs/arquitetura-tempo-real.md.
 */
#include "realtime_loop.h"

#include "angle_lut_usb_commands.h"
#include "control_usb_commands.h"
#include "driver/gptimer.h"
#include "driver/i2c_master.h"
#include "engine_angle_kalman.h"
#include "engine_current_sense.h"
#include "esp_angle_lut.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_rt_diagnostics.h"
#include "esp_timer.h"
#include "esp_timeseries_recorder.h"
#include "esp_timeseries_usb_transport.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "kalman.h"
#include "motor_controller.h"
#include "mt6701.h"
#include "realtime_telemetry.h"
#include <stdatomic.h>

/* Core allocation: keep console/system work away from the control core. */
#define CONTROL_CORE_ID 1

/*
 * Hardware scheduling and communication. The GPTimer runs at an exact 1 MHz
 * and alternates 333, 333 and 334 ticks. Each group therefore lasts exactly
 * 1000 us: three estimator updates and one PID update per millisecond.
 */
#define I2C_PORT_NUM I2C_NUM_0
#define TIMER_RESOLUTION_HZ 1000000U
#define SENSOR_SHORT_PERIOD_TICKS 333U
#define SENSOR_LONG_PERIOD_TICKS 334U
#define SENSOR_PERIODS_PER_PATTERN 3U
#define SENSOR_PATTERN_TICKS                                                   \
  ((2U * SENSOR_SHORT_PERIOD_TICKS) + SENSOR_LONG_PERIOD_TICKS)
#define SENSOR_PERIOD_US_NEAREST                                               \
  ((1000000U + (REALTIME_SENSOR_RATE_HZ / 2U)) / REALTIME_SENSOR_RATE_HZ)
#define SENSOR_DEADLINE_US                                                     \
  ((1000000U + REALTIME_SENSOR_RATE_HZ - 1U) / REALTIME_SENSOR_RATE_HZ)
#define CONTROL_DIVIDER (REALTIME_SENSOR_RATE_HZ / REALTIME_CONTROL_RATE_HZ)

/* Profiling limits identify which stage is consuming the 333.333 us cycle. */
#define DIAG_I2C_BUDGET_US 250U
#define DIAG_KALMAN_BUDGET_US 25U
#define DIAG_CONTROL_BUDGET_US 150U
#define DIAG_SNAPSHOT_BUDGET_US 150U

/* Filter tuning reference and unit conversions. */
#define KALMAN_REFERENCE_RATE_HZ 1000.0f
#define DEGREES_PER_SECOND_TO_RPM (1.0f / 6.0f)
#define MICROSECONDS_TO_SECONDS (1.0f / 1000000.0f)
#define MILLIAMPERES_TO_AMPERES (1.0f / 1000.0f)

/* Tagged current values retain five 16-bit channels and the full capture. */
#define CAPTURE_CURRENT_VALID_LIMIT_MA 32000
#define CAPTURE_CURRENT_BRAKE_I16 32760
#define CAPTURE_CURRENT_COAST_I16 32761
#define CAPTURE_CURRENT_FAULT_R_I16 32762
#define CAPTURE_CURRENT_FAULT_L_I16 32763
#define CAPTURE_CURRENT_FAULT_BOTH_I16 32764
#define CAPTURE_CURRENT_UNAVAILABLE_I16 32765

/* FreeRTOS task resources. */
#define REALTIME_TASK_STACK_SIZE 4096U
#define REALTIME_TASK_PRIORITY (configMAX_PRIORITIES - 1)

#if CONFIG_FREERTOS_NUMBER_OF_CORES < 2
#error "The deterministic control loop requires both ESP32-S3 CPU cores"
#endif
#if !CONFIG_GPTIMER_ISR_HANDLER_IN_IRAM ||                                     \
    !CONFIG_GPTIMER_CTRL_FUNC_IN_IRAM || !CONFIG_GPTIMER_OBJ_CACHE_SAFE
#error                                                                         \
    "The fractional GPTimer ISR requires its handler, control API and object in internal memory"
#endif

_Static_assert(TIMER_RESOLUTION_HZ == 1000000U,
               "Fractional scheduler ticks must represent microseconds");
_Static_assert(SENSOR_PATTERN_TICKS == 1000U,
               "Three estimator intervals must total exactly one millisecond");
_Static_assert(SENSOR_PERIODS_PER_PATTERN == CONTROL_DIVIDER,
               "One fractional pattern must contain one control period");
_Static_assert(REALTIME_SENSOR_RATE_HZ % REALTIME_CONTROL_RATE_HZ == 0,
               "Sensor rate must be divisible by control rate");
_Static_assert(REALTIME_DIAG_STAGE_COUNT <= ESP_RT_DIAG_MAX_STAGES,
               "Application declares too many diagnostic stages");
_Static_assert(REALTIME_DIAG_EVENT_COUNT <= ESP_RT_DIAG_MAX_EVENTS,
               "Application declares too many diagnostic events");
_Static_assert(REALTIME_DIAG_INTERVAL_COUNT <= ESP_RT_DIAG_MAX_INTERVALS,
               "Application declares too many diagnostic intervals");

static const char *TAG = "REALTIME_LOOP";

/**
 * @brief Scale one native MT6701 count into the configured LUT resolution.
 *
 * Called internally by initialize_sensor_and_filter() and realtime_task(). The
 * integer conversion is allocation-free and assumes the LUT full scale is a
 * power of two, as required by esp_angle_lut.
 *
 * @param mt6701_counts Wrapped native sensor count in
 * [0, MT6701_COUNTS_PER_REVOLUTION).
 * @return Wrapped LUT-domain count in [0, ESP_ANGLE_LUT_FULL_SCALE_COUNTS).
 */
static uint16_t mt6701_to_lut_counts(uint16_t mt6701_counts) {
  /* Scale block: use uint32 intermediates, then mask the configured wrap bit.
   */
  uint32_t scaled =
      ((uint32_t)mt6701_counts * ESP_ANGLE_LUT_FULL_SCALE_COUNTS) /
      MT6701_COUNTS_PER_REVOLUTION;
  return (uint16_t)(scaled & (ESP_ANGLE_LUT_FULL_SCALE_COUNTS - 1U));
}

/**
 * @brief Convert one wrapped LUT-domain count to mechanical degrees.
 *
 * Called internally by initialize_sensor_and_filter() and realtime_task() after
 * esp_angle_lut_apply().
 *
 * @return Angle in degrees over the LUT's [0, 360) domain.
 */
static float lut_counts_to_degrees(uint16_t angle_counts) {
  return (float)angle_counts *
         (360.0f / (float)ESP_ANGLE_LUT_FULL_SCALE_COUNTS);
}

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

/*
 * Mutable ISR context kept in internal DRAM. gptimer_set_alarm_action() checks
 * that its configuration is internal-memory resident when its control path is
 * placed in IRAM.
 */
typedef struct {
  TaskHandle_t realtime_task;        /**< Task notified at every alarm. */
  gptimer_alarm_config_t next_alarm; /**< Internal-memory rearm descriptor. */
  uint8_t fractional_phase; /**< Position in the 333/333/334 pattern. */
} sampling_timer_context_t;

static DRAM_ATTR sampling_timer_context_t sampling_timer_context;

typedef enum {
  /** No recorder ARM awaits profile selection. */
  CAPTURE_REQUEST_NONE = 0,
  /** Start the alternating-reference PID profile. */
  CAPTURE_REQUEST_CLOSED_LOOP,
  /** Start the open-loop angle-calibration profile. */
  CAPTURE_REQUEST_CALIBRATION,
} capture_request_t;

static atomic_int pending_capture_request;
/* The USB task alone edits requested_closed_loop_config. Before publishing a
 * CLOSED_LOOP request with release ordering, it copies a coherent snapshot to
 * pending_closed_loop_config; the real-time task reads that snapshot only
 * after acquiring the request. */
static motor_controller_config_t requested_closed_loop_config;
static motor_controller_config_t pending_closed_loop_config;

/**
 * @brief Atomically associate a successful recorder ARM with a test profile.
 *
 * Called internally by arm_closed_loop_capture() and
 * arm_calibration_capture() in the USB task. The release store publishes an
 * optional copied configuration to realtime_task(), which consumes it with an
 * acquire exchange.
 *
 * @return Result from esp_timeseries_arm(); no request is published on error.
 */
static esp_err_t arm_capture_request(uint32_t sample_rate_hz,
                                     capture_request_t request,
                                     const motor_controller_config_t *config) {
  /* Recorder block: reserve the fixed buffer before changing control state. */
  esp_err_t error = esp_timeseries_arm(sample_rate_hz);
  if (error == ESP_OK) {
    /* Payload block: copy volatile gains before publishing their request. */
    if (config != NULL) {
      pending_closed_loop_config = *config;
    }
    /* Publish the start request only after the recorder accepted this ARM. */
    atomic_store_explicit(&pending_capture_request, request,
                          memory_order_release);
  }
  return error;
}

/**
 * @brief Adapt the generic ARM callback to the closed-loop request path.
 *
 * Called by the USB transport task through transport_config.arm_handler. The
 * unused context is reserved by the generic callback signature.
 */
static esp_err_t arm_closed_loop_capture(uint32_t sample_rate_hz,
                                         void *context) {
  (void)context;
  return arm_capture_request(sample_rate_hz, CAPTURE_REQUEST_CLOSED_LOOP,
                             &requested_closed_loop_config);
}

/**
 * @brief Adapt CAL START to recorder ARM plus open-loop profile selection.
 *
 * Called by angle_lut_usb_command_handler() through its configured callback.
 */
static esp_err_t arm_calibration_capture(uint32_t sample_rate_hz,
                                         void *context) {
  (void)context;
  return arm_capture_request(sample_rate_hz, CAPTURE_REQUEST_CALIBRATION, NULL);
}

static angle_lut_usb_command_context_t angle_lut_command_context = {
    .calibration_arm_handler = arm_calibration_capture,
    .calibration_arm_context = NULL,
};

static control_usb_command_context_t control_command_context = {
    .configuration = &requested_closed_loop_config,
};

/**
 * @brief Chain application CONTROL and CAL namespaces for the USB transport.
 *
 * Called by the generic transport task through its command-handler callback.
 * CONTROL receives first refusal; any other command is offered to CAL. The
 * function performs no work in the deterministic task.
 */
static bool
application_usb_command_handler(const char *command,
                                const esp_timeseries_usb_command_io_t *io,
                                void *context) {
  /* Dispatch block: contexts have static storage and are selected per family.
   */
  (void)context;
  if (control_usb_command_handler(command, io, &control_command_context)) {
    return true;
  }
  return angle_lut_usb_command_handler(command, io, &angle_lut_command_context);
}

enum {
  /** Kalman speed estimate scaled as 0.1 RPM/count. */
  CAPTURE_CHANNEL_SPEED_RPM,
  /** Signed current or tagged bridge/fault state. */
  CAPTURE_CHANNEL_CURRENT_A,
  /** Signed bridge command scaled as 0.01 percent/count. */
  CAPTURE_CHANNEL_CONTROL_PERCENT,
  /** Active reference scaled as 0.1 RPM/count. */
  CAPTURE_CHANNEL_REFERENCE_RPM,
  /** Raw MT6701 angle scaled as 0.01 degree/count with 180 degree offset. */
  CAPTURE_CHANNEL_ANGLE_DEG,
  /** Fixed channel count used to size every sample. */
  CAPTURE_CHANNEL_COUNT,
};

static const esp_timeseries_channel_t capture_channels[] = {
    [CAPTURE_CHANNEL_SPEED_RPM] = {.name = "speed",
                                   .unit = "rpm",
                                   .scale = 0.1f,
                                   .offset = 0.0f},
    [CAPTURE_CHANNEL_CURRENT_A] = {.name = "current",
                                   .unit = "A",
                                   .scale = 0.001f,
                                   .offset = 0.0f,
                                   .encoding = "bts7960-current-v1"},
    [CAPTURE_CHANNEL_CONTROL_PERCENT] = {.name = "control",
                                         .unit = "percent",
                                         .scale = 0.01f,
                                         .offset = 0.0f},
    [CAPTURE_CHANNEL_REFERENCE_RPM] = {.name = "reference",
                                       .unit = "rpm",
                                       .scale = 0.1f,
                                       .offset = 0.0f},
    [CAPTURE_CHANNEL_ANGLE_DEG] = {.name = "angle_raw",
                                   .unit = "deg",
                                   .scale = 0.01f,
                                   .offset = 180.0f},
};

/**
 * @brief Convert a reserved int16 current code to its float recorder input.
 *
 * Called internally only by capture_current_for_driver(). Reusing the channel
 * scale makes esp_timeseries_record_f32() quantize back to the exact reserved
 * code without allocating another channel.
 */
static float tagged_current_value(int16_t code) {
  return (float)code * capture_channels[CAPTURE_CHANNEL_CURRENT_A].scale;
}

/**
 * @brief Select signed current or a reserved status code for one capture row.
 *
 * Called internally only by realtime_task() at 1 kHz. It correlates the latest
 * dual-current frame with the bridge mode/direction: R_IS is positive drive,
 * negated L_IS is reverse drive, while faults, BRAKE, COAST, and unavailable
 * states use reserved int16 tags. It performs no ADC transaction itself.
 *
 * @param motor Initialized bridge instance whose cached state is inspected.
 * @return Current in amperes within +/-32 A, or a float that encodes one
 * reserved int16 status after recorder scaling.
 */
static float capture_current_for_driver(const struct engine_config *motor) {
#if CONFIG_ENGINE_CURRENT_SENSE_ENABLE
  /* Measurement block: require a complete latest dual-channel frame. */
  engine_current_sense_measurement_t measurement = {0};
  if (!engine_current_sense_get_latest_measurement(&measurement)) {
    return tagged_current_value(CAPTURE_CURRENT_UNAVAILABLE_I16);
  }

  /* Fault block: fault tags take precedence over bridge mode and current. */
  const uint32_t r_bit =
      ENGINE_CURRENT_SENSE_CHANNEL_BIT(ENGINE_CURRENT_SENSE_CHANNEL_R_IS);
  const uint32_t l_bit =
      ENGINE_CURRENT_SENSE_CHANNEL_BIT(ENGINE_CURRENT_SENSE_CHANNEL_L_IS);
  uint32_t fault_mask = measurement.fault_mask & (r_bit | l_bit);
  if (fault_mask == (r_bit | l_bit)) {
    return tagged_current_value(CAPTURE_CURRENT_FAULT_BOTH_I16);
  }
  if ((fault_mask & r_bit) != 0U) {
    return tagged_current_value(CAPTURE_CURRENT_FAULT_R_I16);
  }
  if ((fault_mask & l_bit) != 0U) {
    return tagged_current_value(CAPTURE_CURRENT_FAULT_L_I16);
  }

  /* Bridge-state block: non-drive modes cannot be interpreted as signed
   * high-side motor current using the BTS7960 IS outputs. */
  engine_driver_state_t driver_state = {0};
  if (engine_driver_get_state(motor, &driver_state) != ESP_OK) {
    return tagged_current_value(CAPTURE_CURRENT_UNAVAILABLE_I16);
  }
  if (driver_state.mode == ENGINE_DRIVER_MODE_BRAKE) {
    return tagged_current_value(CAPTURE_CURRENT_BRAKE_I16);
  }
  if (driver_state.mode == ENGINE_DRIVER_MODE_COAST) {
    return tagged_current_value(CAPTURE_CURRENT_COAST_I16);
  }
  if (driver_state.mode != ENGINE_DRIVER_MODE_DRIVE ||
      (driver_state.direction != 1 && driver_state.direction != -1)) {
    return tagged_current_value(CAPTURE_CURRENT_UNAVAILABLE_I16);
  }

  /* Direction block: select the conducting high-side sense channel and require
   * its validity bit before consuming the converted milliamperes. */
  engine_current_sense_channel_t channel =
      driver_state.direction > 0 ? ENGINE_CURRENT_SENSE_CHANNEL_R_IS
                                 : ENGINE_CURRENT_SENSE_CHANNEL_L_IS;
  uint32_t channel_bit = ENGINE_CURRENT_SENSE_CHANNEL_BIT(channel);
  if ((measurement.valid_mask & channel_bit) == 0U) {
    return tagged_current_value(CAPTURE_CURRENT_UNAVAILABLE_I16);
  }
  /* Encoding block: clamp away from reserved tags, then apply torque sign. */
  int32_t milliamps = measurement.channel_milliamps[channel];
  if (milliamps > CAPTURE_CURRENT_VALID_LIMIT_MA) {
    milliamps = CAPTURE_CURRENT_VALID_LIMIT_MA;
  }
  if (driver_state.direction < 0) {
    milliamps = -milliamps;
  }
  return (float)milliamps * MILLIAMPERES_TO_AMPERES;
#else
  (void)motor;
  return tagged_current_value(CAPTURE_CURRENT_UNAVAILABLE_I16);
#endif
}

/* ============================ 3 KHZ SCHEDULER ============================ */

/**
 * @brief Rearm the fractional scheduler and wake the deterministic task.
 *
 * GPTimer calls this private ISR after start_sampling_timer() registers it. It
 * schedules the next absolute alarm using the repeating
 * 333/333/334 us pattern, timestamps this event, notifies the already-created
 * task, and requests an immediate context switch. Sensor access and all
 * floating-point work deliberately remain outside ISR.
 *
 * @param timer GPTimer instance whose next alarm is reprogrammed.
 * @param event_data ISR-owned event containing the absolute alarm value.
 * @param user_context Internal-DRAM sampling_timer_context_t.
 * @return true when a higher-priority task was awakened and an ISR-exit context
 * switch should occur.
 */
static bool IRAM_ATTR sampling_timer_callback(
    gptimer_handle_t timer, const gptimer_alarm_event_data_t *event_data,
    void *user_context) {
  /* Timestamp block: capture wake origin before rearm/notification overhead. */
  sampling_timer_context_t *context = (sampling_timer_context_t *)user_context;

  esp_rt_diag_isr_capture(&last_timer_isr_time_us);

  /*
   * The first alarm occurs at 333 us. Subsequent deltas are 333, 334 and 333
   * us, producing absolute alarms at 333, 666, 1000, 1333, 1666, 2000...
   * Basing the next deadline on alarm_value prevents ISR latency from drifting
   * the sampling phase.
   */
  uint32_t next_delta = context->fractional_phase == 1U
                            ? SENSOR_LONG_PERIOD_TICKS
                            : SENSOR_SHORT_PERIOD_TICKS;
  context->fractional_phase =
      context->fractional_phase == (SENSOR_PERIODS_PER_PATTERN - 1U)
          ? 0U
          : (uint8_t)(context->fractional_phase + 1U);
  context->next_alarm.alarm_count = event_data->alarm_value + next_delta;
  (void)gptimer_set_alarm_action(timer, &context->next_alarm);

  /* Notification block: counting notifications expose coalesced/missed releases
   * to esp_rt_diag_cycle_begin_from_isr(). */
  BaseType_t high_priority_task_woken = pdFALSE;
  vTaskNotifyGiveFromISR(context->realtime_task, &high_priority_task_woken);
  return high_priority_task_woken == pdTRUE;
}

/**
 * @brief Create the sensor path and seed the Kalman filter from real angle.
 *
 * Called internally only by realtime_task() before current acquisition and the
 * scheduler start. It creates the I2C bus/device, initializes MT6701, applies
 * the installed LUT to the cached initial count, and initializes Kalman. The
 * handles remain owned by realtime_loop_context_t for application lifetime.
 *
 * Create the I2C bus and MT6701 from Core 1, obtain the first valid angle, then
 * initialize the Kalman state at that angle. Starting from the current angle
 * avoids a large artificial innovation during the first filter update.
 *
 * @param context Static loop context receiving I2C, sensor, and filter state.
 * @param initial_angle_deg Destination for corrected initial angle in degrees.
 * @return ESP_OK on success or an ESP-IDF/MT6701 initialization error. Partial
 * bus resources remain application-owned because the runtime cannot proceed.
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

  /* Device block: attach the fixed-address MT6701 at the Kconfig bus speed. */
  i2c_device_config_t device_config = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = MT6701_I2C_ADDRESS,
      .scl_speed_hz = CONFIG_APP_I2C_CLOCK_HZ,
  };
  ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(context->i2c_bus,
                                                &device_config,
                                                &context->i2c_device),
                      TAG, "Could not add MT6701 to I2C bus");

  /* Seed-measurement block: use the driver's initialized cache, then apply the
   * same count-domain conversion and LUT path used during normal acquisition.
   */
  uint16_t initial_raw_angle_counts = 0U;
  ESP_RETURN_ON_ERROR(mt6701_init(&context->sensor, context->i2c_device), TAG,
                      "Could not initialize MT6701");
  ESP_RETURN_ON_ERROR(
      mt6701_get_last_angle_counts(&context->sensor, &initial_raw_angle_counts),
      TAG, "Could not get initial MT6701 angle");
  uint16_t initial_lut_counts = mt6701_to_lut_counts(initial_raw_angle_counts);
  *initial_angle_deg =
      lut_counts_to_degrees(esp_angle_lut_apply(initial_lut_counts));

  /*
   * Q values were originally tuned for a 1 kHz update. Scale their per-update
   * contribution because the estimator runs at a different rate. R is the
   * angle-measurement variance and therefore is not rate-scaled here.
   */
  const float q_rate_scale =
      KALMAN_REFERENCE_RATE_HZ / (float)REALTIME_SENSOR_RATE_HZ;
  kalman_3d_config_t filter_config = {
      .q_theta = 0.001f * q_rate_scale,
      .q_omega = 5.0f * q_rate_scale,
      .q_alpha = 100.0f * q_rate_scale,
      .r = 0.002f,
  };
  /* Filter block: initialize zero speed/acceleration around measured angle. */
  kalman_3d_init(&context->filter, *initial_angle_deg, &filter_config);
  return ESP_OK;
}

/**
 * @brief Create and start the exact-average 3 kHz GPTimer scheduler.
 *
 * Called internally only by realtime_task(). It registers
 * sampling_timer_callback(), initializes its internal-DRAM context, and starts
 * the first one-shot alarm. The ISR rearms all subsequent absolute deadlines.
 *
 * @param realtime_task Existing Core 1 task notified by the ISR.
 * @param timer Destination receiving the created GPTimer handle.
 * @return ESP_OK on success or an error propagated by the GPTimer driver.
 *
 * Configure a 1 MHz GPTimer with an initial one-shot alarm at 333 us. The ISR
 * subsequently rearms absolute alarms with the 333/333/334 us pattern.
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

  /* Callback block: initialize all mutable ISR state before registration. */
  gptimer_event_callbacks_t callbacks = {
      .on_alarm = sampling_timer_callback,
  };
  sampling_timer_context = (sampling_timer_context_t){
      .realtime_task = realtime_task,
      .next_alarm =
          {
              .reload_count = 0,
              .flags.auto_reload_on_alarm = false,
          },
      .fractional_phase = 0U,
  };
  ESP_RETURN_ON_ERROR(gptimer_register_event_callbacks(*timer, &callbacks,
                                                       &sampling_timer_context),
                      TAG, "Could not register GPTimer callback");

  /* Start block: program the first 333 us deadline, enable, then start. */
  gptimer_alarm_config_t alarm_config = {
      .alarm_count = SENSOR_SHORT_PERIOD_TICKS,
      .reload_count = 0,
      .flags.auto_reload_on_alarm = false,
  };
  ESP_RETURN_ON_ERROR(gptimer_set_alarm_action(*timer, &alarm_config), TAG,
                      "Could not configure GPTimer alarm");
  ESP_RETURN_ON_ERROR(gptimer_enable(*timer), TAG, "Could not enable GPTimer");
  return gptimer_start(*timer);
}

/**
 * @brief Assemble and publish a coherent application telemetry snapshot.
 *
 * Called internally only by realtime_task() after a diagnostics window closes.
 * It reads task-owned estimator/controller state, asks other components for
 * copy snapshots, and calls realtime_telemetry_publish(). It never formats or
 * prints logs on Core 1.
 *
 * Assemble one coherent snapshot from live Core 1 state. The telemetry module
 * owns the queue and all Core 0 formatting; this function only maps loop state
 * into the application payload after the instrumentation snapshot is closed.
 *
 * @param context Core 1 loop state containing sensor and estimator.
 * @param diagnostics Completed generic diagnostics snapshot to publish.
 * @param measured_angle_deg Latest LUT-corrected measurement in degrees.
 * @param controller Current control/profile instance to snapshot.
 */
static void publish_telemetry_snapshot(
    realtime_loop_context_t *context, const esp_rt_diag_snapshot_t *diagnostics,
    float measured_angle_deg, const motor_controller_t *controller) {
  /* Turn count comes from MT6701 tracking; speed comes from Kalman below. */
  int32_t total_turns = 0;
  mt6701_get_total_turns(&context->sensor, &total_turns);
  motor_controller_status_t controller_status = {0};
  motor_controller_get_status(controller, &controller_status);

  /* Application-state block: convert estimator angular units and copy PID
   * status into a self-contained payload. */
  realtime_telemetry_payload_t telemetry = {
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
      .open_loop_stage = controller_status.open_loop_stage,
      .open_loop_test = controller_status.open_loop_test,
      .open_loop_test_started = controller_status.open_loop_test_started,
  };
  /* Component-state block: append independent recorder and current snapshots;
   * failures are represented explicitly instead of blocking publication. */
  telemetry.recorder_status_valid =
      esp_timeseries_get_status(&telemetry.recorder) == ESP_OK;
#if CONFIG_ENGINE_CURRENT_SENSE_ENABLE
  engine_current_sense_take_dual_snapshot(&telemetry.current_sense);
  engine_current_sense_get_latest_dual_frame(&telemetry.current_frame);
#endif
  /* Publication block: reporter copies this stack payload before returning. */
  realtime_telemetry_publish(diagnostics, &telemetry);
}

/* ============= CORE 1 ACQUISITION, ESTIMATION AND CONTROL ============== */

/**
 * @brief Run acquisition, estimation, PID, actuation, capture, and diagnostics.
 *
 * Created internally only by realtime_loop_start() and permanently pinned as
 * the highest-priority application task on Core 1. It never returns during
 * normal operation; fatal initialization failures make the motor safe and
 * delete the task.
 *
 * Highest-priority application task, permanently pinned to Core 1.
 * Fast path on every timer event (3 kHz): sensor -> measured dt -> Kalman.
 * Divided path on every third event (1 kHz): controller -> motor driver.
 * Side path at the configured window: copy diagnostics to the Core 0 logger.
 *
 * @param argument Static realtime_loop_context_t passed during task creation.
 */
static void realtime_task(void *argument) {
  /* Controller/sensor initialization block: construct all task-owned state
   * before enabling any periodic interrupt source. */
  realtime_loop_context_t *context = (realtime_loop_context_t *)argument;
  motor_controller_t controller;
  motor_controller_init(&controller);

  float measured_angle_deg = 0.0f;
  float raw_angle_deg = 0.0f;
  esp_err_t err = initialize_sensor_and_filter(context, &measured_angle_deg);
  raw_angle_deg = measured_angle_deg;
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Real-time initialization failed: %s", esp_err_to_name(err));
    engine_driver_set_speed(context->motor, 0.0f);
    vTaskDelete(NULL);
    return;
  }

  ESP_LOGI(TAG,
           "Control on Core %d: sensor/Kalman=%u Hz control=%u Hz "
           "I2C clock=%u Hz synchronous initial=%.3f deg PID closed-loop",
           xPortGetCoreID(), REALTIME_SENSOR_RATE_HZ, REALTIME_CONTROL_RATE_HZ,
           CONFIG_APP_I2C_CLOCK_HZ, measured_angle_deg);

#if CONFIG_ENGINE_CURRENT_SENSE_ENABLE
  /*
   * Start the 1 ms ADC frames immediately before the 3 kHz GPTimer. Both run
   * independently, but this gives the current frames a stable initial phase
   * relative to every third timer alarm without burdening the control path.
   */
  err = engine_current_sense_start();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Current acquisition initialization failed: %s",
             esp_err_to_name(err));
    engine_driver_set_speed(context->motor, 0.0f);
    vTaskDelete(NULL);
    return;
  }
#endif

  /* Scheduler-start block: no periodic work begins until every required input
   * subsystem is ready; rollback leaves the bridge non-driving. */
  gptimer_handle_t sampling_timer = NULL;
  err = start_sampling_timer(xTaskGetCurrentTaskHandle(), &sampling_timer);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Sampling timer initialization failed: %s",
             esp_err_to_name(err));
    engine_driver_set_speed(context->motor, 0.0f);
#if CONFIG_ENGINE_CURRENT_SENSE_ENABLE
    engine_current_sense_stop();
#endif
    vTaskDelete(NULL);
    return;
  }

  /* Divider and timestamps persist across diagnostic window resets. */
  uint32_t control_divider = 0;
  /* dt timestamps use actual completion/start instants rather than nominal dt.
   */
  int64_t last_sample_time_us = context->sensor.last_timestamp_us;
  int64_t last_control_time_us = last_sample_time_us;
  float motor_output_percent = 0.0f;
#if CONFIG_ESP_RT_DIAGNOSTICS_ENABLE
  esp_rt_diag_t diagnostics;
  esp_rt_diag_t *diagnostics_ptr = &diagnostics;
  const esp_rt_diag_config_t diagnostics_config = {
      /* The exact period is 333.333 us. Diagnostics use the nearest integer
       * for the displayed expected rate and the ceiling for the deadline. */
      .expected_period_us = SENSOR_PERIOD_US_NEAREST,
      .deadline_us = SENSOR_DEADLINE_US,
      .window_duration_us =
          (uint32_t)CONFIG_ESP_RT_DIAGNOSTICS_WINDOW_MS * 1000U,
      .stage_count = REALTIME_DIAG_STAGE_COUNT,
      .event_count = REALTIME_DIAG_EVENT_COUNT,
      .interval_count = REALTIME_DIAG_INTERVAL_COUNT,
      .stage_budget_us =
          {
              [REALTIME_DIAG_STAGE_I2C] = DIAG_I2C_BUDGET_US,
              [REALTIME_DIAG_STAGE_KALMAN] = DIAG_KALMAN_BUDGET_US,
              [REALTIME_DIAG_STAGE_CONTROL] = DIAG_CONTROL_BUDGET_US,
              [REALTIME_DIAG_STAGE_SNAPSHOT] = DIAG_SNAPSHOT_BUDGET_US,
          },
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

  /* Deterministic loop: every iteration corresponds to one timer notification
   * or a coalesced set that diagnostics records as missed releases. */
  while (true) {
    /* Scheduler: wait for the 3 kHz GPTimer notification. */
    uint32_t pending_events = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    esp_rt_diag_cycle_begin_from_isr(diagnostics_ptr, &last_timer_isr_time_us,
                                     pending_events);

    /* Acquisition: perform one complete synchronous burst read per cycle. */
    int64_t i2c_start_us = esp_rt_diag_stage_begin();
    err = mt6701_update(&context->sensor);
    int64_t i2c_end_us = esp_timer_get_time();
    esp_rt_diag_stage_record(diagnostics_ptr, REALTIME_DIAG_STAGE_I2C,
                             i2c_start_us, i2c_end_us);

    if (err == ESP_OK) {
      /* Timestamp the sample when the I2C transaction has completed. */
      int64_t sample_time_us = context->sensor.last_timestamp_us;
      uint32_t sample_dt_us = (uint32_t)(sample_time_us - last_sample_time_us);
      last_sample_time_us = sample_time_us;
      esp_rt_diag_interval(diagnostics_ptr, REALTIME_DIAG_INTERVAL_SAMPLE,
                           sample_dt_us);

      /* Estimation: update angle, speed and acceleration at 3 kHz. */
      int64_t kalman_start_us = esp_rt_diag_stage_begin();
      float sample_dt = (float)sample_dt_us * MICROSECONDS_TO_SECONDS;
      /*
       * get_last reads the value cached by mt6701_update(); it causes no second
       * I2C transaction. Reject pathological dt values after a long disruption.
       */
      uint16_t raw_angle_counts = 0U;
      if (mt6701_get_last_angle_counts(&context->sensor, &raw_angle_counts) ==
              ESP_OK &&
          sample_dt > 0.0f && sample_dt < 0.1f) {
        raw_angle_deg = mt6701_counts_to_degrees(raw_angle_counts);
        uint16_t lut_angle_counts = mt6701_to_lut_counts(raw_angle_counts);
        measured_angle_deg =
            lut_counts_to_degrees(esp_angle_lut_apply(lut_angle_counts));
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

    /* Control: every third estimator cycle produces the 1 kHz motor action. */
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

      /* Profile-request block: synchronize USB ARM with controller reset only
       * inside the 1 kHz owner task. */
      esp_timeseries_state_t recorder_state = esp_timeseries_get_state();
      /*
       * ARMED may be shorter than one control period: the USB task can arm the
       * recorder after the state read above, and record_f32() can advance it to
       * CAPTURING before the next 1 kHz iteration. Consume the explicit request
       * in either active state so starting the test does not depend on
       * observing that transient state.
       */
      if (recorder_state == ESP_TIMESERIES_STATE_ARMED ||
          recorder_state == ESP_TIMESERIES_STATE_CAPTURING) {
        capture_request_t request = atomic_exchange_explicit(
            &pending_capture_request, CAPTURE_REQUEST_NONE,
            memory_order_acq_rel);
        if (request == CAPTURE_REQUEST_CALIBRATION) {
          motor_controller_start_open_loop_test(&controller);
        } else if (request == CAPTURE_REQUEST_CLOSED_LOOP) {
          (void)motor_controller_start_closed_loop_test(
              &controller, &pending_closed_loop_config);
        }
      }

      /* Control-law block: convert Kalman speed units and advance one PID or
       * open-loop profile update using the actual elapsed control interval. */
      float estimated_speed_rpm =
          context->filter.x[1] * DEGREES_PER_SECOND_TO_RPM;
      motor_output_percent = motor_controller_update(
          &controller, estimated_speed_rpm,
          (float)control_dt_us * MICROSECONDS_TO_SECONDS);
      /*
       * A zero controller command selects dynamic braking for this
       * application: both motor terminals are clamped to the low side. Keep
       * COAST as a distinct driver operation for initialization and fail-safe
       * shutdown paths.
       */
      if (motor_output_percent == 0.0f) {
        engine_driver_brake(context->motor);
      } else {
        engine_driver_set_speed(context->motor, motor_output_percent);
      }

      /* Capture block: snapshot post-actuation state into the fixed
       * five-channel sample; recorder decimation decides whether this call
       * stores it. */
      motor_controller_status_t controller_status = {0};
      motor_controller_get_status(&controller, &controller_status);
      float current_amperes = capture_current_for_driver(context->motor);
      const float capture_values[CAPTURE_CHANNEL_COUNT] = {
          [CAPTURE_CHANNEL_SPEED_RPM] = estimated_speed_rpm,
          [CAPTURE_CHANNEL_CURRENT_A] = current_amperes,
          [CAPTURE_CHANNEL_CONTROL_PERCENT] = motor_output_percent,
          [CAPTURE_CHANNEL_REFERENCE_RPM] = controller_status.reference_rpm,
          [CAPTURE_CHANNEL_ANGLE_DEG] = raw_angle_deg,
      };
      esp_timeseries_record_f32(capture_values, control_start_us);
      esp_rt_diag_event(diagnostics_ptr, REALTIME_DIAG_EVENT_CONTROL_UPDATE,
                        1U);
      esp_rt_diag_stage_end(diagnostics_ptr, REALTIME_DIAG_STAGE_CONTROL,
                            control_stage_start_us);
    }

    /* Instrumentation: measure this cycle without printing from Core 1. */
    int64_t processing_end_us = esp_rt_diag_cycle_end_now(diagnostics_ptr);

    /* Snapshot and application payload are copied only when the window ends. */
    if (esp_rt_diag_snapshot_due(diagnostics_ptr, processing_end_us)) {
      int64_t snapshot_start_us = esp_rt_diag_stage_begin();
      esp_rt_diag_snapshot_t diagnostics_snapshot;
      bool snapshot_taken = false;
      if (esp_rt_diag_try_take_snapshot(diagnostics_ptr, processing_end_us,
                                        &diagnostics_snapshot,
                                        &snapshot_taken) == ESP_OK &&
          snapshot_taken) {
        publish_telemetry_snapshot(context, &diagnostics_snapshot,
                                   measured_angle_deg, &controller);
      }
      esp_rt_diag_stage_end(diagnostics_ptr, REALTIME_DIAG_STAGE_SNAPSHOT,
                            snapshot_start_us);
    }
  }
}

/**
 * @brief Initialize services and create the complete dual-core runtime.
 *
 * Called externally only by app_main(). See realtime_loop.h for lifetime,
 * prerequisites, and return behavior.
 *
 * Public startup entry point. Create telemetry first and real-time second so
 * every published snapshot has a consumer. All objects have application
 * lifetime after successful startup.
 */
esp_err_t realtime_loop_start(struct engine_config *motor) {
  /* Preconditions/state block: retain the initialized static motor and seed
   * both requested and pending volatile configurations with firmware defaults.
   */
  if (motor == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  loop_context.motor = motor;
  atomic_init(&pending_capture_request, CAPTURE_REQUEST_NONE);
  motor_controller_get_default_config(&requested_closed_loop_config);
  pending_closed_loop_config = requested_closed_loop_config;

  /* Service block: initialize correction, recorder metadata/buffer, and USB
   * protocol before any producer can generate samples. */
  esp_err_t lut_err = esp_angle_lut_init();
  if (lut_err != ESP_OK && lut_err != ESP_ERR_INVALID_STATE) {
    return lut_err;
  }

  const esp_timeseries_config_t recorder_config = {
      .producer_rate_hz = REALTIME_CONTROL_RATE_HZ,
      .channel_count = CAPTURE_CHANNEL_COUNT,
      .channels = capture_channels,
  };
  esp_err_t recorder_err = esp_timeseries_init(&recorder_config);
  if (recorder_err != ESP_OK) {
    return recorder_err;
  }

  const esp_timeseries_usb_transport_config_t transport_config = {
      .arm_handler = arm_closed_loop_capture,
      .arm_handler_context = NULL,
      .command_handler = application_usb_command_handler,
      .command_handler_context = NULL,
      .extension_help = CONTROL_USB_COMMAND_HELP "," ANGLE_LUT_USB_COMMAND_HELP,
  };
  esp_err_t err = esp_timeseries_usb_transport_start(&transport_config);
  if (err != ESP_OK) {
    return err;
  }

#if CONFIG_ESP_RT_DIAGNOSTICS_ENABLE
  /* Telemetry block: create the Core 0 consumer before the Core 1 producer. */
  err = realtime_telemetry_start();
  if (err != ESP_OK) {
    esp_timeseries_usb_transport_stop();
    return err;
  }
#endif

  /* Task block: sensor, estimator, and controller share Core 1 at the highest
   * application priority. Roll back previously started task-based services if
   * allocation fails. */
  BaseType_t task_created = xTaskCreatePinnedToCore(
      realtime_task, "motor_realtime", REALTIME_TASK_STACK_SIZE, &loop_context,
      REALTIME_TASK_PRIORITY, NULL, CONTROL_CORE_ID);
  if (task_created != pdPASS) {
#if CONFIG_ESP_RT_DIAGNOSTICS_ENABLE
    realtime_telemetry_stop();
#endif
    esp_timeseries_usb_transport_stop();
    return ESP_ERR_NO_MEM;
  }

  return ESP_OK;
}
