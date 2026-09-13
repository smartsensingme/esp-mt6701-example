#include "motor_controller.h"
#include "sdkconfig.h"
#include <math.h>
#include <stddef.h>

/*
 * Manual PID tuning parameters.
 *
 * Units with speed error in RPM and output in percent:
 *   Kp: % / RPM
 *   Ki: % / (RPM.s)
 *   Kd: %.s / RPM
 *
 * Start conservatively. Adjust the instance configuration and rebuild while
 * observing reference, error, P/I/D terms, and output in telemetry.
 */
static const motor_controller_config_t default_controller_config = {
    .kp = 0.25f,
    .ki = 3.0f,
    .kd = 0.0001f,
    .reference_step_period_s = 2.0f,
};

static const esp_pid_config_t speed_pid_base_config = {
    .output_min = -100.0f,
    .output_max = 100.0f,
    .derivative_filter_tau_s = 0.020f,
    .anti_windup_tracking_time_s = 0.20f,
    .maximum_dt_s = 0.1f,
    .derivative_source = ESP_PID_DERIVATIVE_ON_MEASUREMENT,
};

/* Alternating step profile used for manual closed-loop tuning. */
static const float reference_low_rpm = 600.0f;
static const float reference_high_rpm = 900.0f;

/* Low/high/low stages followed by zero output, synchronized to CAL START. */
static const float open_loop_duty_percent[] = {
    (float)CONFIG_APP_OPEN_LOOP_DUTY_LOW_PERCENT,
    (float)CONFIG_APP_OPEN_LOOP_DUTY_HIGH_PERCENT,
    (float)CONFIG_APP_OPEN_LOOP_DUTY_LOW_PERCENT, 0.0f};
static const float open_loop_stage_period_s =
    (float)CONFIG_APP_OPEN_LOOP_STAGE_SECONDS;

/**
 * @brief Copy the immutable application PID/profile defaults.
 *
 * Called by realtime_loop_start(), motor_controller_init(), and the CONTROL
 * DEFAULTS command path. See motor_controller.h for the public contract.
 */
void motor_controller_get_default_config(motor_controller_config_t *config) {
  /* Copy block: tolerate optional callers that provide no destination. */
  if (config != NULL) {
    *config = default_controller_config;
  }
}

/**
 * @brief Validate all volatile controller parameters and protocol bounds.
 *
 * Called by control_usb_command_handler(), reset/start paths, and external
 * application code through motor_controller.h.
 */
bool motor_controller_config_is_valid(const motor_controller_config_t *config) {
  /* Validation block: short-circuit null, NaN, infinity, and out-of-range
   * values so invalid arithmetic never reaches esp_pid. */
  return config != NULL && isfinite(config->kp) && config->kp >= 0.0f &&
         config->kp <= 100.0f && isfinite(config->ki) && config->ki >= 0.0f &&
         config->ki <= 1000.0f && isfinite(config->kd) && config->kd >= 0.0f &&
         config->kd <= 10.0f && isfinite(config->reference_step_period_s) &&
         config->reference_step_period_s >= 0.1f &&
         config->reference_step_period_s <= 3600.0f;
}

/**
 * @brief Rebuild controller and PID state for a selected operating mode.
 *
 * Called internally by motor_controller_init(),
 * motor_controller_start_closed_loop_test(), and
 * motor_controller_start_open_loop_test(). Callers guarantee non-null pointers
 * and a validated configuration. It calls esp_pid_init().
 */
static void reset_controller(motor_controller_t *controller,
                             motor_controller_mode_t mode,
                             const motor_controller_config_t *configuration) {
  /* State-reset block: aggregate initialization clears elapsed time, counters,
   * flags, output, and embedded PID history in one deterministic assignment. */
  *controller = (motor_controller_t){
      .config = *configuration,
      .reference_rpm = reference_low_rpm,
      .mode = mode,
  };
  /* PID-configuration block: combine fixed safety/dynamic settings with the
   * experiment's volatile gains before initializing the embedded instance. */
  esp_pid_config_t pid_config = speed_pid_base_config;
  pid_config.kp = configuration->kp;
  pid_config.ki = configuration->ki;
  pid_config.kd = configuration->kd;
  (void)esp_pid_init(&controller->pid, &pid_config);
}

/**
 * @brief Initialize an application controller in its safe IDLE state.
 *
 * Called by realtime_task() before timer startup. See motor_controller.h.
 */
void motor_controller_init(motor_controller_t *controller) {
  /* Initialization block: a null pointer is intentionally a no-op. */
  if (controller != NULL) {
    reset_controller(controller, MOTOR_CONTROLLER_MODE_IDLE,
                     &default_controller_config);
  }
}

/**
 * @brief Validate and start a fresh closed-loop step-response experiment.
 *
 * Called by realtime_task() after consuming a closed-loop capture request.
 */
bool motor_controller_start_closed_loop_test(
    motor_controller_t *controller,
    const motor_controller_config_t *configuration) {
  /* Precondition block: leave existing controller state intact on rejection. */
  if (controller == NULL || !motor_controller_config_is_valid(configuration)) {
    return false;
  }
  /* Start block: reset PID history and begin at the 600 RPM reference. */
  reset_controller(controller, MOTOR_CONTROLLER_MODE_CLOSED_LOOP,
                   configuration);
  return true;
}

/**
 * @brief Start the deterministic low/high/low open-loop calibration profile.
 *
 * Called by realtime_task() after consuming CAL START. See the public header.
 */
void motor_controller_start_open_loop_test(motor_controller_t *controller) {
  if (controller != NULL) {
    /* Configuration block: preserve a valid volatile configuration so later
     * state remains coherent; fall back defensively if it was corrupted. */
    motor_controller_config_t configuration = controller->config;
    if (!motor_controller_config_is_valid(&configuration)) {
      configuration = default_controller_config;
    }
    /* Start block: reset profile/PID state and mark calibration as running. */
    reset_controller(controller, MOTOR_CONTROLLER_MODE_OPEN_LOOP_TEST,
                     &configuration);
    controller->open_loop_test_started = true;
  }
}

/**
 * @brief Advance the selected profile and calculate its signed motor command.
 *
 * Called only by realtime_task() at the 1 kHz divided rate. See
 * motor_controller.h for units and return limits.
 */
float motor_controller_update(motor_controller_t *controller,
                              float measured_speed_rpm, float dt) {
  /* Interface block: a missing instance maps to the safe zero command. */
  if (controller == NULL) {
    return 0.0f;
  }

  /* Mode block: invalid dt freezes profile time; IDLE always remains zero. */
  bool valid_dt = dt > 0.0f && dt < 0.1f;
  if (controller->mode == MOTOR_CONTROLLER_MODE_IDLE) {
    controller->output_percent = 0.0f;
    return 0.0f;
  }
  if (controller->mode == MOTOR_CONTROLLER_MODE_OPEN_LOOP_TEST) {
    /* Open-loop timing block: valid dt is below 0.1 s, whereas each stage lasts
     * at least one second. Consequently, one update can cross at most one stage
     * boundary; retaining the excess time prevents cumulative phase drift. */
    if (valid_dt && controller->open_loop_stage < 3U) {
      controller->profile_elapsed_s += dt;
      if (controller->profile_elapsed_s >= open_loop_stage_period_s) {
        controller->profile_elapsed_s -= open_loop_stage_period_s;
        controller->open_loop_stage++;
      }
    }

    /* Open-loop output block: no speed reference participates in calibration.
     */
    controller->reference_rpm = 0.0f;
    controller->output_percent =
        open_loop_duty_percent[controller->open_loop_stage];
    return controller->output_percent;
  }

  /* Closed-loop profile block: dt is below 0.1 s and the validated reference
   * period is at least 0.1 s. One update therefore crosses at most one
   * boundary; subtracting instead of clearing retains the fractional excess
   * time. */
  if (valid_dt) {
    controller->profile_elapsed_s += dt;
    if (controller->profile_elapsed_s >=
        controller->config.reference_step_period_s) {
      controller->profile_elapsed_s -=
          controller->config.reference_step_period_s;
      controller->reference_step_count++;
      controller->high_reference_active = !controller->high_reference_active;
      controller->reference_rpm = controller->high_reference_active
                                      ? reference_high_rpm
                                      : reference_low_rpm;
    }
  }

  /* PID block: esp_pid performs derivative filtering, saturation, and
   * back-calculation anti-windup. Any API error falls back to zero output. */
  if (esp_pid_update(&controller->pid, controller->reference_rpm,
                     measured_speed_rpm, dt,
                     &controller->output_percent) != ESP_OK) {
    controller->output_percent = 0.0f;
  }
  return controller->output_percent;
}

/**
 * @brief Copy controller, profile, and PID state into a transportable snapshot.
 *
 * Called by realtime_task() and publish_telemetry_snapshot(). See the public
 * header for ownership and null behavior.
 */
void motor_controller_get_status(const motor_controller_t *controller,
                                 motor_controller_status_t *status) {
  /* Interface block: never partially write an invalid destination. */
  if (controller == NULL || status == NULL) {
    return;
  }

  /* Snapshot block: obtain PID terms first, then publish one aggregate value.
   */
  esp_pid_state_t pid_state = {0};
  (void)esp_pid_get_state(&controller->pid, &pid_state);
  *status = (motor_controller_status_t){
      .reference_rpm = controller->reference_rpm,
      .error_rpm = pid_state.error,
      .proportional_term = pid_state.proportional_term,
      .integral_term = pid_state.integral_term,
      .derivative_term = pid_state.derivative_term,
      .output_percent = controller->output_percent,
      .reference_step_count = controller->reference_step_count,
      .open_loop_stage = controller->open_loop_stage,
      .open_loop_test =
          controller->mode == MOTOR_CONTROLLER_MODE_OPEN_LOOP_TEST,
      .open_loop_test_started = controller->open_loop_test_started,
  };
}
