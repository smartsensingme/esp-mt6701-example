#ifndef MOTOR_CONTROLLER_H_
#define MOTOR_CONTROLLER_H_

#include <stdbool.h>
#include <stdint.h>

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
  uint8_t open_loop_stage;
  bool high_reference_active;
  bool previous_speed_valid;
} motor_controller_t;

typedef struct {
  float reference_rpm;
  float error_rpm;
  float proportional_term;
  float integral_term;
  float derivative_term;
  float output_percent;
  uint8_t open_loop_stage;
  bool open_loop_test;
} motor_controller_status_t;

/** Initialize the selected test profile and its controller state. */
void motor_controller_init(motor_controller_t *controller);

/**
 * @brief Calculate the 1 kHz motor command.
 *
 * In the temporary open-loop test, the output runs at 60, 80, and 90 percent
 * for 20 seconds each and then remains at zero (COAST). When that test is
 * disabled in Kconfig, the reference alternates between 600 and 900 RPM.
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

#endif /* MOTOR_CONTROLLER_H_ */
