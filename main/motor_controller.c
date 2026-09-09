#include "motor_controller.h"
#include "sdkconfig.h"
#include <stddef.h>

/*
 * Manual PID tuning parameters.
 *
 * Units with speed error in RPM and output in percent:
 *   Kp: % / RPM
 *   Ki: % / (RPM.s)
 *   Kd: %.s / RPM
 *
 * Start conservatively. Adjust these three static variables and rebuild while
 * observing reference, error, P/I/D terms, and output in telemetry.
 */
static float pid_kp = 0.25f;
static float pid_ki = 3.0f;
static float pid_kd = 0.0001f;

/* Alternating step profile used for manual closed-loop tuning. */
static float reference_low_rpm = 600.0f;
static float reference_high_rpm = 900.0f;
static float reference_step_period_s = 4.0f;

/* Low-pass time constant applied before using the speed derivative. */
static float derivative_filter_tau_s = 0.020f;

/*
 * Back-calculation tracking time. Smaller values pull the integral term toward
 * the realizable actuator output more quickly while the command is saturated.
 */
static float anti_windup_tracking_time_s = 0.20f;

#if CONFIG_APP_MOTOR_OPEN_LOOP_TEST
/* Low/high/low stages followed by COAST, synchronized to recorder ARM. */
static const float open_loop_duty_percent[] = {
    (float)CONFIG_APP_OPEN_LOOP_DUTY_LOW_PERCENT,
    (float)CONFIG_APP_OPEN_LOOP_DUTY_HIGH_PERCENT,
    (float)CONFIG_APP_OPEN_LOOP_DUTY_LOW_PERCENT, 0.0f};
static const float open_loop_stage_period_s =
    (float)CONFIG_APP_OPEN_LOOP_STAGE_SECONDS;
#endif

#define MOTOR_OUTPUT_MIN_PERCENT 0.0f
#define MOTOR_OUTPUT_MAX_PERCENT 100.0f

static float clamp(float value, float minimum, float maximum) {
  if (value < minimum) {
    return minimum;
  }
  if (value > maximum) {
    return maximum;
  }
  return value;
}

static void reset_controller(motor_controller_t *controller,
                             motor_controller_mode_t mode) {
  *controller = (motor_controller_t){
      .reference_rpm = reference_low_rpm,
      .mode = mode,
  };
}

void motor_controller_init(motor_controller_t *controller) {
  if (controller != NULL) {
    reset_controller(controller, MOTOR_CONTROLLER_MODE_IDLE);
  }
}

void motor_controller_start_closed_loop_test(motor_controller_t *controller) {
  if (controller != NULL) {
    reset_controller(controller, MOTOR_CONTROLLER_MODE_CLOSED_LOOP);
  }
}

void motor_controller_start_open_loop_test(motor_controller_t *controller) {
#if CONFIG_APP_MOTOR_OPEN_LOOP_TEST
  if (controller != NULL) {
    reset_controller(controller, MOTOR_CONTROLLER_MODE_OPEN_LOOP_TEST);
    controller->open_loop_test_started = true;
  }
#else
  (void)controller;
#endif
}

float motor_controller_update(motor_controller_t *controller,
                              float measured_speed_rpm, float dt) {
  if (controller == NULL) {
    return 0.0f;
  }

  bool valid_dt = dt > 0.0f && dt < 0.1f;
  if (controller->mode == MOTOR_CONTROLLER_MODE_IDLE) {
    controller->output_percent = 0.0f;
    return 0.0f;
  }
#if CONFIG_APP_MOTOR_OPEN_LOOP_TEST
  if (controller->mode == MOTOR_CONTROLLER_MODE_OPEN_LOOP_TEST) {
    if (valid_dt && controller->open_loop_stage < 3U) {
      controller->profile_elapsed_s += dt;
      while (controller->profile_elapsed_s >= open_loop_stage_period_s &&
             controller->open_loop_stage < 3U) {
        controller->profile_elapsed_s -= open_loop_stage_period_s;
        controller->open_loop_stage++;
      }
    }

    controller->reference_rpm = 0.0f;
    controller->error_rpm = 0.0f;
    controller->proportional_term = 0.0f;
    controller->integral_term = 0.0f;
    controller->derivative_term = 0.0f;
    controller->output_percent =
        open_loop_duty_percent[controller->open_loop_stage];
    return controller->output_percent;
  }
#endif

  if (valid_dt) {
    controller->profile_elapsed_s += dt;
    while (controller->profile_elapsed_s >= reference_step_period_s) {
      controller->profile_elapsed_s -= reference_step_period_s;
      controller->reference_step_count++;
      controller->high_reference_active = !controller->high_reference_active;
      controller->reference_rpm = controller->high_reference_active
                                      ? reference_high_rpm
                                      : reference_low_rpm;
    }
  }

  controller->error_rpm = controller->reference_rpm - measured_speed_rpm;
  controller->proportional_term = pid_kp * controller->error_rpm;

  if (valid_dt && controller->previous_speed_valid) {
    float raw_speed_derivative_rpm_s =
        (measured_speed_rpm - controller->previous_speed_rpm) / dt;
    float derivative_alpha = dt / (derivative_filter_tau_s + dt);
    controller->filtered_speed_derivative_rpm_s +=
        derivative_alpha * (raw_speed_derivative_rpm_s -
                            controller->filtered_speed_derivative_rpm_s);
  }
  controller->previous_speed_rpm = measured_speed_rpm;
  controller->previous_speed_valid = true;

  /* Derivative on measurement avoids a kick when the reference changes. */
  controller->derivative_term =
      -pid_kd * controller->filtered_speed_derivative_rpm_s;

  float candidate_integral = controller->integral_term;
  if (valid_dt) {
    candidate_integral += pid_ki * controller->error_rpm * dt;
  }

  /*
   * Back-calculation anti-windup: the difference between the requested and
   * realizable actuator outputs drives the integral term back toward a state
   * consistent with the saturated command. Inside the actuator range the
   * tracking error is zero and this reduces to ordinary integral action.
   */
  float candidate_output = controller->proportional_term + candidate_integral +
                           controller->derivative_term;
  float realizable_output = clamp(candidate_output, MOTOR_OUTPUT_MIN_PERCENT,
                                  MOTOR_OUTPUT_MAX_PERCENT);
  if (valid_dt) {
    float tracking_error = realizable_output - candidate_output;
    candidate_integral += tracking_error * dt / anti_windup_tracking_time_s;
  }
  controller->integral_term = candidate_integral;

  float unsaturated_output = controller->proportional_term +
                             controller->integral_term +
                             controller->derivative_term;
  controller->output_percent = clamp(
      unsaturated_output, MOTOR_OUTPUT_MIN_PERCENT, MOTOR_OUTPUT_MAX_PERCENT);
  return controller->output_percent;
}

void motor_controller_get_status(const motor_controller_t *controller,
                                 motor_controller_status_t *status) {
  if (controller == NULL || status == NULL) {
    return;
  }

  *status = (motor_controller_status_t){
      .reference_rpm = controller->reference_rpm,
      .error_rpm = controller->error_rpm,
      .proportional_term = controller->proportional_term,
      .integral_term = controller->integral_term,
      .derivative_term = controller->derivative_term,
      .output_percent = controller->output_percent,
      .reference_step_count = controller->reference_step_count,
      .open_loop_stage = controller->open_loop_stage,
      .open_loop_test =
          controller->mode == MOTOR_CONTROLLER_MODE_OPEN_LOOP_TEST,
      .open_loop_test_started = controller->open_loop_test_started,
  };
}

bool motor_controller_get_next_reference_step(
    const motor_controller_t *controller, float *seconds_remaining,
    uint32_t *step_id) {
  if (controller == NULL || seconds_remaining == NULL || step_id == NULL) {
    return false;
  }
  if (controller->mode != MOTOR_CONTROLLER_MODE_CLOSED_LOOP) {
    return false;
  }
  float remaining = reference_step_period_s - controller->profile_elapsed_s;
  *seconds_remaining = remaining > 0.0f ? remaining : 0.0f;
  *step_id = controller->reference_step_count + 1U;
  return true;
}
