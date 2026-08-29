#ifndef ENGINE_ANGLE_KALMAN_H_
#define ENGINE_ANGLE_KALMAN_H_

#include "kalman.h"

/**
 * @brief Update a 3D angle Kalman filter with circular wrap-around handling.
 *
 * @param filter Kalman filter state.
 * @param measured_angle Measured angle in degrees.
 * @param dt Elapsed time since the previous sample, in seconds.
 */
void engine_angle_kalman_3d_update(struct kalman_3d *filter,
                                   float measured_angle, float dt);

#endif /* ENGINE_ANGLE_KALMAN_H_ */
