#ifndef MOTOR_CONTROLLER_H_
#define MOTOR_CONTROLLER_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  MOTOR_CONTROLLER_MODE_IDLE = 0,
  MOTOR_CONTROLLER_MODE_CLOSED_LOOP,
  MOTOR_CONTROLLER_MODE_OPEN_LOOP_TEST,
} motor_controller_mode_t;

typedef struct {
  float reference_rpm;
  float error_rpm;
  float integral_term;
  float filtered_speed_derivative_rpm_s;
  float previous_speed_rpm;
  float profile_elapsed_s;
  float proportional_term;
  float derivative_term;
  float output_percent;
  uint32_t reference_step_count;
  uint8_t open_loop_stage;
  motor_controller_mode_t mode;
  bool high_reference_active;
  bool open_loop_test_started;
  bool previous_speed_valid;
} motor_controller_t;

typedef struct {
  float reference_rpm;
  float error_rpm;
  float proportional_term;
  float integral_term;
  float derivative_term;
  float output_percent;
  uint32_t reference_step_count;
  uint8_t open_loop_stage;
  bool open_loop_test;
  bool open_loop_test_started;
} motor_controller_status_t;

/** Initialize the selected test profile and its controller state. */
void motor_controller_init(motor_controller_t *controller);

/** Start a fresh 600/900 RPM closed-loop experiment. */
void motor_controller_start_closed_loop_test(motor_controller_t *controller);

/** Start or restart the Kconfig-enabled calibration profile. */
void motor_controller_start_open_loop_test(motor_controller_t *controller);

/**
 * @brief Calculate the 1 kHz motor command.
 *
 * Output remains in COAST until a test is explicitly started. Closed-loop mode
 * alternates the reference between 600 and 900 RPM. Calibration mode follows
 * the Kconfig-selected low/high/low duty profile and returns to COAST.
 *
 * @param controller Private controller state.
 * @param measured_speed_rpm Kalman speed estimate in RPM.
 * @param dt Actual elapsed time since the previous control update, in seconds.
 * @return Motor command in percent, limited to 0..100.
 */
float motor_controller_update(motor_controller_t *controller,
                              float measured_speed_rpm, float dt);

/** Copy the current reference, error, PID terms, and saturated output. */
void motor_controller_get_status(const motor_controller_t *controller,
                                 motor_controller_status_t *status);

/**
 * Describe the next automatic closed-loop reference transition.
 *
 * @return true in the alternating-reference profile; false in open-loop mode
 * or for invalid arguments.
 */
bool motor_controller_get_next_reference_step(
    const motor_controller_t *controller, float *seconds_remaining,
    uint32_t *step_id);

#endif /* MOTOR_CONTROLLER_H_ */
