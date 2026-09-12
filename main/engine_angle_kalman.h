#ifndef ENGINE_ANGLE_KALMAN_H_
#define ENGINE_ANGLE_KALMAN_H_

#include "kalman.h"

/**
 * @brief Update a 3D angle Kalman filter with circular wrap-around handling.
 *
 * This application-level update is called internally only by realtime_task()
 * after a successful MT6701 acquisition and LUT correction. It performs the
 * complete prediction and measurement-correction steps in place. The function
 * allocates no memory, performs no I/O, and is intended for the deterministic
 * estimator path.
 *
 * @param filter Initialized mutable Kalman state. It must be non-null and owned
 * exclusively by the caller during the update.
 * @param measured_angle Corrected wrapped measurement in degrees, normally in
 * [0, 360).
 * @param dt Positive elapsed sample time in seconds. realtime_task() rejects
 * invalid and abnormally long intervals before calling this function.
 */
void engine_angle_kalman_3d_update(struct kalman_3d *filter,
                                   float measured_angle, float dt);

#endif /* ENGINE_ANGLE_KALMAN_H_ */
