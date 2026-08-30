#include "motor_controller.h"

void motor_controller_init(motor_controller_t *controller,
                           float dummy_output_percent) {
  controller->dummy_output_percent = dummy_output_percent;
}

float motor_controller_update(motor_controller_t *controller,
                              float measured_speed_rpm, float dt) {
  (void)measured_speed_rpm;
  (void)dt;
  return controller->dummy_output_percent;
}
