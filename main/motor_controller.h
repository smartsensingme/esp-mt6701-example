#ifndef MOTOR_CONTROLLER_H_
#define MOTOR_CONTROLLER_H_

typedef struct {
  float dummy_output_percent;
} motor_controller_t;

void motor_controller_init(motor_controller_t *controller,
                           float dummy_output_percent);

/**
 * Temporary fixed-output controller. The speed and dt parameters are already
 * part of the API for the future PID implementation.
 */
float motor_controller_update(motor_controller_t *controller,
                              float measured_speed_rpm, float dt);

#endif /* MOTOR_CONTROLLER_H_ */
