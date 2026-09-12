#ifndef REALTIME_TELEMETRY_H_
#define REALTIME_TELEMETRY_H_

#include "engine_current_sense.h"
#include "esp_err.h"
#include "esp_rt_diagnostics.h"
#include "esp_timeseries_recorder.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum {
  /** Number of successful angle/Kalman updates in the window. */
  REALTIME_DIAG_EVENT_ESTIMATOR_UPDATE,
  /** Number of completed PID/motor/capture updates in the window. */
  REALTIME_DIAG_EVENT_CONTROL_UPDATE,
  /** Number of MT6701 acquisition failures in the window. */
  REALTIME_DIAG_EVENT_SENSOR_ERROR,
  /** Number of application event counters supplied to diagnostics. */
  REALTIME_DIAG_EVENT_COUNT,
} realtime_diag_event_id_t;

typedef enum {
  /** Synchronous MT6701 I2C transaction duration. */
  REALTIME_DIAG_STAGE_I2C,
  /** Cached-angle conversion, LUT correction, and Kalman duration. */
  REALTIME_DIAG_STAGE_KALMAN,
  /** PID, bridge command, current selection, and recorder duration. */
  REALTIME_DIAG_STAGE_CONTROL,
  /** Diagnostics/application snapshot construction duration. */
  REALTIME_DIAG_STAGE_SNAPSHOT,
  /** Number of application stages supplied to diagnostics. */
  REALTIME_DIAG_STAGE_COUNT,
} realtime_diag_stage_id_t;

typedef enum {
  /** Interval between completed MT6701 measurements. */
  REALTIME_DIAG_INTERVAL_SAMPLE,
  /** Interval between successive control calculations. */
  REALTIME_DIAG_INTERVAL_CONTROL,
  /** Number of application interval trackers supplied to diagnostics. */
  REALTIME_DIAG_INTERVAL_COUNT,
} realtime_diag_interval_id_t;

/**
 * @brief Application payload copied from the Core 1 loop to the reporter.
 *
 * The reporter receives a copy alongside esp_rt_diag_snapshot_t, so its task
 * never accesses live sensor, Kalman, controller, or motor objects.
 */
typedef struct {
  /** R_IS/L_IS statistics accumulated over the same diagnostic window. */
  engine_current_sense_dual_snapshot_t current_sense;
  /** Most recent synchronized R_IS/L_IS ADC frame. */
  engine_current_sense_dual_frame_t current_frame;

  /** Recorder state captured at publication time. */
  esp_timeseries_status_t recorder;
  /** True when recorder contains a valid status structure. */
  bool recorder_status_valid;

  /** MT6701 accumulated mechanical turns. */
  int32_t total_turns;
  /** LUT-corrected measured angle in degrees, wrapped to [0, 360). */
  float measured_angle_deg;
  /** Kalman angle estimate in degrees, wrapped to [0, 360). */
  float estimated_angle_deg;
  /** Kalman angular-speed estimate converted to RPM. */
  float estimated_speed_rpm;
  /** Kalman angular-acceleration estimate in RPM/s. */
  float estimated_acceleration_rpm_s;
  /** Active controller reference in RPM. */
  float speed_reference_rpm;
  /** Reference-minus-measurement error in RPM. */
  float speed_error_rpm;
  /** Latest PID proportional contribution in percent. */
  float pid_proportional_term;
  /** Latest PID integral contribution in percent. */
  float pid_integral_term;
  /** Latest PID derivative contribution in percent. */
  float pid_derivative_term;
  /** Latest signed bridge command in percent. */
  float motor_output_percent;
  /** Zero-based calibration profile stage. */
  uint8_t open_loop_stage;
  /** True when the controller is in open-loop calibration mode. */
  bool open_loop_test;
  /** True after CAL START has started the duty sequence. */
  bool open_loop_test_started;
} realtime_telemetry_payload_t;

/**
 * @brief Create the generic reporter and its low-priority Core 0 task.
 *
 * Called externally only by realtime_loop_start() when diagnostics are enabled.
 * It must run from task context before the first publish and may be called only
 * once until realtime_telemetry_stop().
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if already started, or an
 * error propagated by esp_rt_diag_reporter_create().
 */
esp_err_t realtime_telemetry_start(void);

/**
 * @brief Delete telemetry resources after a partial application startup.
 *
 * Called externally by realtime_loop_start() if creation of the control task
 * fails. It is also safe for application shutdown after a successful start.
 * The function leaves the module ready for a subsequent start.
 */
void realtime_telemetry_stop(void);

/**
 * @brief Copy the latest diagnostics and payload to the overwrite-only
 * reporter.
 *
 * This call never waits for the logger. If an older snapshot is still queued,
 * it is replaced because stale telemetry must not delay the control loop.
 *
 * Called internally only by publish_telemetry_snapshot() on Core 1. Both input
 * structures are copied by the reporter before the function returns and need
 * remain valid only for the duration of the call.
 *
 * @param diagnostics Completed generic diagnostics window snapshot.
 * @param application_payload Coherent application state for the same window.
 */
void realtime_telemetry_publish(
    const esp_rt_diag_snapshot_t *diagnostics,
    const realtime_telemetry_payload_t *application_payload);

#endif /* REALTIME_TELEMETRY_H_ */
