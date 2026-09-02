#ifndef REALTIME_TELEMETRY_H_
#define REALTIME_TELEMETRY_H_

#include "engine_current_sense.h"
#include "esp_err.h"
#include "esp_rt_diagnostics.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum {
  REALTIME_DIAG_EVENT_ESTIMATOR_UPDATE,
  REALTIME_DIAG_EVENT_CONTROL_UPDATE,
  REALTIME_DIAG_EVENT_SENSOR_ERROR,
  REALTIME_DIAG_EVENT_COUNT,
} realtime_diag_event_id_t;

typedef enum {
  REALTIME_DIAG_STAGE_I2C,
  REALTIME_DIAG_STAGE_KALMAN,
  REALTIME_DIAG_STAGE_CONTROL,
  REALTIME_DIAG_STAGE_SNAPSHOT,
  REALTIME_DIAG_STAGE_COUNT,
} realtime_diag_stage_id_t;

typedef enum {
  REALTIME_DIAG_INTERVAL_SAMPLE,
  REALTIME_DIAG_INTERVAL_CONTROL,
  REALTIME_DIAG_INTERVAL_COUNT,
} realtime_diag_interval_id_t;

/**
 * @brief Immutable snapshot sent from the Core 1 loop to the Core 0 logger.
 *
 * It combines the reusable timing result with an application-owned
 * instantaneous control payload. The telemetry task receives a copy, so it
 * never accesses live sensor, Kalman, controller, or motor objects.
 */
typedef struct {
  /* Reusable timing/counter snapshot produced by esp_rt_diagnostics. */
  esp_rt_diag_snapshot_t diagnostics;

  /* Application-owned R_IS acquisition statistics for the same window. */
  engine_current_sense_snapshot_t current_sense;
  engine_current_sense_frame_t current_frame;

  /* Application-owned instantaneous state captured at publication time. */
  int32_t total_turns;
  float measured_angle_deg;
  float estimated_angle_deg;
  float estimated_speed_rpm;
  float estimated_acceleration_rpm_s;
  float speed_reference_rpm;
  float speed_error_rpm;
  float pid_proportional_term;
  float pid_integral_term;
  float pid_derivative_term;
  float motor_output_percent;
  uint8_t open_loop_stage;
  bool open_loop_test;
  bool open_loop_test_started;
} realtime_telemetry_snapshot_t;

/**
 * @brief Create the single-slot queue and low-priority Core 0 logger task.
 */
esp_err_t realtime_telemetry_start(void);

/**
 * @brief Delete telemetry resources after a partial application startup.
 */
void realtime_telemetry_stop(void);

/**
 * @brief Copy the latest snapshot to the overwrite-only telemetry queue.
 *
 * This call never waits for the logger. If an older snapshot is still queued,
 * it is replaced because stale telemetry must not delay the control loop.
 *
 */
void realtime_telemetry_publish(const realtime_telemetry_snapshot_t *snapshot);

#endif /* REALTIME_TELEMETRY_H_ */
