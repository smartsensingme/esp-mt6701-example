#ifndef MOTOR_CONTROLLER_H_
#define MOTOR_CONTROLLER_H_

#include <stdbool.h>

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
} motor_controller_status_t;

/** Initialize PID state and select the low reference as the first step. */
void motor_controller_init(motor_controller_t *controller);

/**
 * @brief Calculate the 1 kHz motor command.
 *
 * The reference alternates between two static values every ten seconds. Output
 * is limited to 0..100 percent for unidirectional speed control. Conditional
 * integration prevents windup, and the derivative is taken from the measured
 * speed so a reference step does not produce derivative kick.
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
