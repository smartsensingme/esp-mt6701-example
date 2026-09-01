#ifndef REALTIME_TELEMETRY_H_
#define REALTIME_TELEMETRY_H_

#include "esp_err.h"
#include <stdint.h>

/**
 * @brief Immutable snapshot sent from the Core 1 loop to the Core 0 logger.
 *
 * The real-time loop fills this value from its current plant state and timing
 * instrumentation. The telemetry task receives a copy, so it never accesses
 * live sensor, Kalman, controller, or motor objects.
 */
typedef struct {
  /* Counts collected only during the reported five-second window. */
  uint32_t estimator_updates;
  uint32_t control_updates;
  uint32_t missed_timer_events;
  uint32_t sensor_errors;
  uint32_t deadline_overruns;
  uint32_t window_duration_us;

  /* Maximum execution times measured during that window, all in us. */
  uint32_t max_wake_latency_us;
  uint32_t max_i2c_time_us;
  uint32_t max_kalman_time_us;
  uint32_t max_control_time_us;
  uint32_t max_processing_time_us;
  uint32_t max_cycle_time_us;
  uint32_t lifetime_max_processing_time_us;
  uint32_t previous_telemetry_time_us;

  /* Minimum/maximum actual sampling and control intervals, in us. */
  uint32_t min_sample_dt_us;
  uint32_t max_sample_dt_us;
  uint32_t min_control_dt_us;
  uint32_t max_control_dt_us;

  /* Lifetime counters, including windows intentionally omitted from logs. */
  uint32_t total_estimator_updates;
  uint32_t total_control_updates;
  uint32_t total_missed_timer_events;
  uint32_t total_sensor_errors;

  /* Plant and estimator state captured at publication time. */
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
