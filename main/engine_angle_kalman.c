#include "engine_angle_kalman.h"

/**
 * @brief Predict and correct the application constant-acceleration estimator.
 *
 * Called internally only by realtime_task() through the declaration in
 * engine_angle_kalman.h. It expands the 3x3 matrix operations into scalar
 * expressions to keep the 3 kHz path bounded and allocation-free.
 */
void engine_angle_kalman_3d_update(struct kalman_3d *k, float measured_angle,
                                   float dt) {
  /*
   * Constant-acceleration state model:
   *   x[0] = angle [deg]
   *   x[1] = angular velocity [deg/s]
   *   x[2] = angular acceleration [deg/s^2]
   * The only direct measurement is angle, so H = [1 0 0].
   */

  /* --- 1. PREDICT STEP --- */
  float h = 0.5f * dt * dt;

  /* x_pred = F * x */
  float x_pred_theta = k->x[0] + k->x[1] * dt + k->x[2] * h;
  float x_pred_omega = k->x[1] + k->x[2] * dt;
  float x_pred_alpha = k->x[2];

  /*
   * P_pred = F * P * F^T + Q. The expressions are expanded explicitly to
   * avoid general-purpose matrix operations in the fast estimator path.
   */
  float a0 = k->P[0][0] + dt * k->P[1][0] + h * k->P[2][0];
  float a1 = k->P[0][1] + dt * k->P[1][1] + h * k->P[2][1];
  float a2 = k->P[0][2] + dt * k->P[1][2] + h * k->P[2][2];

  float a4 = k->P[1][1] + dt * k->P[2][1];
  float a5 = k->P[1][2] + dt * k->P[2][2];

  float P_pred_00 = a0 + dt * a1 + h * a2 + k->Q_theta;
  float P_pred_01 = a1 + dt * a2;
  float P_pred_02 = a2;

  float P_pred_10 = P_pred_01;
  float P_pred_11 = a4 + dt * a5 + k->Q_omega;
  float P_pred_12 = a5;

  float P_pred_20 = P_pred_02;
  float P_pred_22 = k->P[2][2] + k->Q_alpha;

  /* --- 2. UPDATE / CORRECT STEP --- */
  float y = measured_angle - x_pred_theta;

  /* Normalize innovation to [-180, 180] degrees at the circular boundary. */
  while (y > 180.0f) {
    y -= 360.0f;
  }
  while (y < -180.0f) {
    y += 360.0f;
  }

  /* Innovation covariance and Kalman gain. */
  float S = P_pred_00 + k->R;
  float K_0 = P_pred_00 / S;
  float K_1 = P_pred_10 / S;
  float K_2 = P_pred_20 / S;

  /* Correct state. */
  k->x[0] = x_pred_theta + K_0 * y;
  k->x[1] = x_pred_omega + K_1 * y;
  k->x[2] = x_pred_alpha + K_2 * y;

  /* Keep the estimated angle within [0, 360). */
  while (k->x[0] >= 360.0f) {
    k->x[0] -= 360.0f;
  }
  while (k->x[0] < 0.0f) {
    k->x[0] += 360.0f;
  }

  /*
   * Correct covariance: P = (I - K * H) * P_pred. Symmetric terms are copied
   * from their counterparts instead of recomputing equivalent expressions.
   */
  k->P[0][0] = (1.0f - K_0) * P_pred_00;
  k->P[0][1] = (1.0f - K_0) * P_pred_01;
  k->P[0][2] = (1.0f - K_0) * P_pred_02;

  k->P[1][0] = k->P[0][1];
  k->P[1][1] = P_pred_11 - K_1 * P_pred_01;
  k->P[1][2] = P_pred_12 - K_1 * P_pred_02;

  k->P[2][0] = k->P[0][2];
  k->P[2][1] = k->P[1][2];
  k->P[2][2] = P_pred_22 - K_2 * P_pred_02;
}
