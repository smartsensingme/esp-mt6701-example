/*
 * Non-real-time presentation layer.
 *
 * Core 1 only calls realtime_telemetry_publish() to copy a snapshot and its
 * application payload. The reusable reporter owns the queue, Core 0 task,
 * generic timing logs, and quiet/log-affected window classification. This file
 * formats only motor-application state.
 */
#include "realtime_telemetry.h"

#include "esp_log.h"
#include "esp_rt_diagnostics_reporter.h"
#include "esp_timeseries_recorder.h"
#include "freertos/FreeRTOS.h"
#include <inttypes.h>
#include <stdbool.h>

#define TELEMETRY_CORE_ID 0
#define TELEMETRY_TASK_STACK_SIZE 4096U
#define TELEMETRY_TASK_PRIORITY 1
#if CONFIG_APP_RT_DIAGNOSTICS_APPLICATION_REPORT_COMPACT ||                    \
    CONFIG_APP_RT_DIAGNOSTICS_APPLICATION_REPORT_FULL
static const char *TAG = "REALTIME_LOOP";
#endif

/* Singleton application reporter. Only startup/shutdown modify the handle;
 * Core 1 only reads it while publishing overwrite-only snapshots. */
static esp_rt_diag_reporter_handle_t telemetry_reporter;

/* Names use the exact enum order exported by realtime_telemetry.h. */
static const char *const stage_names[REALTIME_DIAG_STAGE_COUNT] = {
    [REALTIME_DIAG_STAGE_I2C] = "i2c",
    [REALTIME_DIAG_STAGE_KALMAN] = "kalman",
    [REALTIME_DIAG_STAGE_CONTROL] = "control",
    [REALTIME_DIAG_STAGE_SNAPSHOT] = "snapshot",
};

static const char *const event_names[REALTIME_DIAG_EVENT_COUNT] = {
    [REALTIME_DIAG_EVENT_ESTIMATOR_UPDATE] = "estimator_update",
    [REALTIME_DIAG_EVENT_CONTROL_UPDATE] = "control_update",
    [REALTIME_DIAG_EVENT_SENSOR_ERROR] = "sensor_error",
};

static const char *const interval_names[REALTIME_DIAG_INTERVAL_COUNT] = {
    [REALTIME_DIAG_INTERVAL_SAMPLE] = "sample",
    [REALTIME_DIAG_INTERVAL_CONTROL] = "control",
};

#if CONFIG_APP_RT_DIAGNOSTICS_APPLICATION_REPORT_COMPACT ||                    \
    CONFIG_APP_RT_DIAGNOSTICS_APPLICATION_REPORT_FULL
#if CONFIG_APP_RT_DIAGNOSTICS_APPLICATION_REPORT_FULL &&                       \
    CONFIG_ENGINE_CURRENT_SENSE_ENABLE
/**
 * @brief Format window and latest-frame diagnostics for one current channel.
 *
 * Called internally only by log_application_snapshot() in full-report builds
 * with current sensing enabled. It runs on the low-priority Core 0 reporter
 * task and may perform calibrated conversions and ESP logging freely.
 *
 * @param current Accumulated statistics for one R_IS or L_IS channel.
 * @param frame Latest synchronized ADC frame for the same channel.
 * @param channel Channel identifier used for conversion and display name.
 * @param window_class Stable reporter-owned label valid during the call.
 */
static void log_current_channel(const engine_current_sense_snapshot_t *current,
                                const engine_current_sense_frame_t *frame,
                                engine_current_sense_channel_t channel,
                                const char *window_class) {
  /* Window-conversion block: convert raw ADC extrema and normal-only mean to
   * calibrated millivolts only when their source sample populations exist. */
  const char *channel_name = engine_current_sense_channel_name(channel);
  int adc_average_mv = 0;
  int adc_minimum_mv = 0;
  int adc_maximum_mv = 0;
  int normal_average_mv = 0;
  bool voltage_valid =
      current->samples > 0U &&
      engine_current_sense_channel_raw_to_millivolts(
          channel, current->raw_average, &adc_average_mv) == ESP_OK &&
      engine_current_sense_channel_raw_to_millivolts(
          channel, current->raw_minimum, &adc_minimum_mv) == ESP_OK &&
      engine_current_sense_channel_raw_to_millivolts(
          channel, current->raw_maximum, &adc_maximum_mv) == ESP_OK;
  bool normal_current_valid =
      current->normal_samples > 0U &&
      engine_current_sense_channel_raw_to_millivolts(
          channel, current->normal_raw_average, &normal_average_mv) == ESP_OK;
  /* Window-report block: fault windows retain threshold activity and separate
   * normal-current estimates; clean windows use a compact informational line.
   */
  if (voltage_valid) {
    float fault_percent =
        current->samples > 0U
            ? 100.0f * (float)current->fault_samples / (float)current->samples
            : 0.0f;
    if (current->fault_samples > 0U || current->fault_active) {
      ESP_LOGW(TAG,
               "%s[%s]: FAULT ADC=%d/%d/%d mV fault=%" PRIu32 "/%" PRIu32
               " (%.1f%%) entries=%" PRIu32
               " active=%s I_IS_max=%.2f mA normal_current=%s%.2f A "
               "invalid=%" PRIu32 " overflow=%" PRIu32 " read_errors=%" PRIu32,
               channel_name, window_class, adc_average_mv, adc_minimum_mv,
               adc_maximum_mv, current->fault_samples, current->samples,
               fault_percent, current->fault_entries,
               current->fault_active ? "yes" : "no",
               engine_current_sense_adc_to_i_is_milliamperes(adc_maximum_mv),
               normal_current_valid ? "" : "n/a ",
               normal_current_valid
                   ? engine_current_sense_adc_to_amperes(normal_average_mv)
                   : 0.0f,
               current->invalid_results, current->pool_overflows,
               current->read_errors);
    } else {
      ESP_LOGI(TAG,
               "%s[%s]: normal ADC=%d/%d/%d mV current_equiv=%.2f A "
               "samples=%" PRIu32 " invalid=%" PRIu32 " overflow=%" PRIu32
               " read_errors=%" PRIu32 " (avg/min/max)",
               channel_name, window_class, adc_average_mv, adc_minimum_mv,
               adc_maximum_mv,
               normal_current_valid
                   ? engine_current_sense_adc_to_amperes(normal_average_mv)
                   : 0.0f,
               current->samples, current->invalid_results,
               current->pool_overflows, current->read_errors);
    }
  } else {
    ESP_LOGW(TAG,
             "%s[%s]: no calibrated samples; samples=%" PRIu32
             " invalid=%" PRIu32 " overflow=%" PRIu32 " read_errors=%" PRIu32,
             channel_name, window_class, current->samples,
             current->invalid_results, current->pool_overflows,
             current->read_errors);
  }

  /* Frame-conversion block: the latest frame complements window statistics and
   * is omitted entirely when calibrated conversion is unavailable. */
  int frame_average_mv = 0;
  int frame_median_mv = 0;
  int frame_normal_average_mv = 0;
  bool frame_valid =
      frame->samples > 0U &&
      engine_current_sense_channel_raw_to_millivolts(
          channel, frame->raw_average, &frame_average_mv) == ESP_OK &&
      engine_current_sense_channel_raw_to_millivolts(
          channel, frame->raw_median, &frame_median_mv) == ESP_OK;
  if (!frame_valid) {
    return;
  }

  /* Frame-report block: distinguish fault-contaminated and clean frames. */
  bool frame_normal_valid = frame->normal_samples > 0U &&
                            engine_current_sense_channel_raw_to_millivolts(
                                channel, frame->normal_raw_average,
                                &frame_normal_average_mv) == ESP_OK;
  if (frame->fault_samples > 0U || frame->fault_active) {
    ESP_LOGW(TAG,
             "%s-frame[%s]: FAULT sequence=%" PRIu32
             " ADC mean/median=%d/%d mV fault=%" PRIu32 "/%" PRIu32
             " entries=%" PRIu32 " active=%s normal_current=%s%.2f A",
             channel_name, window_class, frame->sequence, frame_average_mv,
             frame_median_mv, frame->fault_samples, frame->samples,
             frame->fault_entries, frame->fault_active ? "yes" : "no",
             frame_normal_valid ? "" : "n/a ",
             frame_normal_valid
                 ? engine_current_sense_adc_to_amperes(frame_normal_average_mv)
                 : 0.0f);
  } else {
    ESP_LOGI(TAG,
             "%s-frame[%s]: normal sequence=%" PRIu32 " samples=%" PRIu32
             " ADC mean/median=%d/%d mV current_equiv=%.2f/%.2f A",
             channel_name, window_class, frame->sequence, frame->samples,
             frame_average_mv, frame_median_mv,
             engine_current_sense_adc_to_amperes(frame_average_mv),
             engine_current_sense_adc_to_amperes(frame_median_mv));
  }
}
#endif

/**
 * @brief Format motor-application state after each generic timing report.
 *
 * Registered by realtime_telemetry_start() and called only by the reusable
 * reporter task on Core 0. The reporter owns both input copies for the call.
 * Compile-time options select compact, full, or no application report.
 */
static void log_application_snapshot(const esp_rt_diag_snapshot_t *diagnostics,
                                     const void *application_payload,
                                     esp_rt_diag_window_class_t window_class,
                                     void *context) {
  /* Callback-context block: this formatter needs only the copied payload and
   * generic window classification; generic metrics are printed by reporter. */
  (void)diagnostics;
  (void)context;
  const realtime_telemetry_payload_t *telemetry =
      (const realtime_telemetry_payload_t *)application_payload;
  const char *window_class_name = esp_rt_diag_window_class_name(window_class);

#if CONFIG_APP_RT_DIAGNOSTICS_APPLICATION_REPORT_COMPACT
  /* Compact block: emit one bounded state/recorder line per window. */
  const esp_timeseries_status_t *recorder = &telemetry->recorder;
  const char *mode = telemetry->open_loop_test ? "OPEN_LOOP" : "PID";
  uint8_t phase = telemetry->open_loop_test && telemetry->open_loop_test_started
                      ? telemetry->open_loop_stage + 1U
                      : 0U;
  ESP_LOGI(TAG,
           "state[%s]: mode=%s phase=%" PRIu8
           " speed=%.1f ref=%.1f output=%.1f%% recorder=%s:%zu/%zu",
           window_class_name, mode, phase, telemetry->estimated_speed_rpm,
           telemetry->speed_reference_rpm, telemetry->motor_output_percent,
           telemetry->recorder_status_valid
               ? esp_timeseries_state_name(recorder->state)
               : "ERROR",
           telemetry->recorder_status_valid ? recorder->sample_count : 0U,
           telemetry->recorder_status_valid ? recorder->sample_capacity : 0U);
  return;
#endif

#if CONFIG_APP_RT_DIAGNOSTICS_APPLICATION_REPORT_FULL
  /* Full state block: open-loop and PID modes expose different meaningful
   * fields while retaining common estimator data and units. */
  if (telemetry->open_loop_test) {
    const char *stage_name = !telemetry->open_loop_test_started ? "WAIT_ARM"
                             : telemetry->open_loop_stage < 3U  ? "DRIVE"
                                                                : "COAST";
    uint8_t displayed_stage = telemetry->open_loop_test_started
                                  ? telemetry->open_loop_stage + 1U
                                  : 0U;
    ESP_LOGI(TAG,
             "state[%s]: angle=%.3f/%.3f deg speed=%.3f RPM "
             "accel=%.3f RPM/s turns=%" PRId32 " mode=OPEN_LOOP stage=%" PRIu8
             "(%s) output=%.1f%%",
             window_class_name, telemetry->measured_angle_deg,
             telemetry->estimated_angle_deg, telemetry->estimated_speed_rpm,
             telemetry->estimated_acceleration_rpm_s, telemetry->total_turns,
             displayed_stage, stage_name, telemetry->motor_output_percent);
  } else {
    ESP_LOGI(TAG,
             "state[%s]: angle=%.3f/%.3f deg speed=%.3f RPM "
             "accel=%.3f RPM/s turns=%" PRId32
             " reference=%.1f RPM error=%.1f RPM "
             "pid=%.2f/%.2f/%.2f%% output=%.1f%%",
             window_class_name, telemetry->measured_angle_deg,
             telemetry->estimated_angle_deg, telemetry->estimated_speed_rpm,
             telemetry->estimated_acceleration_rpm_s, telemetry->total_turns,
             telemetry->speed_reference_rpm, telemetry->speed_error_rpm,
             telemetry->pid_proportional_term, telemetry->pid_integral_term,
             telemetry->pid_derivative_term, telemetry->motor_output_percent);
  }
#if CONFIG_APP_RT_DIAGNOSTICS_APPLICATION_REPORT_FULL &&                       \
    CONFIG_ENGINE_CURRENT_SENSE_ENABLE
  /* Current block: format both sense outputs outside the real-time core. */
  for (size_t channel = 0; channel < ENGINE_CURRENT_SENSE_CHANNEL_COUNT;
       channel++) {
    log_current_channel(&telemetry->current_sense.channel[channel],
                        &telemetry->current_frame.channel[channel],
                        (engine_current_sense_channel_t)channel,
                        window_class_name);
  }
#endif

  /* Recorder block: derive capacity duration only when sampling is active. */
  const esp_timeseries_status_t *recorder = &telemetry->recorder;
  if (telemetry->recorder_status_valid) {
    float capacity_seconds =
        recorder->sample_rate_hz > 0U
            ? (float)recorder->sample_capacity / (float)recorder->sample_rate_hz
            : 0.0f;
    ESP_LOGI(TAG,
             "recorder[%s]: state=%s id=%" PRIu32 " rate=%" PRIu32
             " Hz samples=%zu/%zu "
             "duration=%.3f s buffer=%zu bytes [0x%" PRIxPTR ",0x%" PRIxPTR ")",
             window_class_name, esp_timeseries_state_name(recorder->state),
             recorder->capture_id, recorder->sample_rate_hz,
             recorder->sample_count, recorder->sample_capacity,
             capacity_seconds, recorder->buffer_bytes, recorder->buffer_begin,
             recorder->buffer_end);
  }
#endif
}
#endif /* compact or full application report */

/**
 * @brief Create and configure the application diagnostics reporter.
 *
 * Called externally only by realtime_loop_start(). See the public header for
 * lifecycle and return behavior.
 */
esp_err_t realtime_telemetry_start(void) {
  /* Singleton block: prevent replacing a live reporter and leaking its task. */
  if (telemetry_reporter != NULL) {
    return ESP_ERR_INVALID_STATE;
  }

  /* Configuration block: bind application enum names, optional payload
   * callback, compact generic format, and low-priority Core 0 execution. */
  const esp_rt_diag_reporter_config_t config = {
      .stage_names = stage_names,
      .stage_name_count = REALTIME_DIAG_STAGE_COUNT,
      .event_names = event_names,
      .event_name_count = REALTIME_DIAG_EVENT_COUNT,
      .interval_names = interval_names,
      .interval_name_count = REALTIME_DIAG_INTERVAL_COUNT,
#if CONFIG_APP_RT_DIAGNOSTICS_APPLICATION_REPORT_COMPACT ||                    \
    CONFIG_APP_RT_DIAGNOSTICS_APPLICATION_REPORT_FULL
      .application_payload_size = sizeof(realtime_telemetry_payload_t),
      .report_callback = log_application_snapshot,
#else
      .application_payload_size = 0U,
      .report_callback = NULL,
#endif
      .report_format = ESP_RT_DIAG_REPORT_COMPACT,
      .task_stack_size = TELEMETRY_TASK_STACK_SIZE,
      .task_priority = TELEMETRY_TASK_PRIORITY,
      .task_core = TELEMETRY_CORE_ID,
  };
  /* Creation block: the reusable component owns task and queue allocations. */
  return esp_rt_diag_reporter_create(&config, &telemetry_reporter);
}

/**
 * @brief Delete the application reporter and clear its singleton handle.
 *
 * Called by realtime_loop_start() during rollback and available for orderly
 * application shutdown through realtime_telemetry.h.
 */
void realtime_telemetry_stop(void) {
  /* Delete is delegated even for null; the component defines safe cleanup. */
  esp_rt_diag_reporter_delete(telemetry_reporter);
  telemetry_reporter = NULL;
}

/**
 * @brief Publish an overwrite-only diagnostics/application snapshot pair.
 *
 * Called only by publish_telemetry_snapshot() on Core 1. See the public header
 * for copy lifetime and non-blocking semantics.
 */
void realtime_telemetry_publish(
    const esp_rt_diag_snapshot_t *diagnostics,
    const realtime_telemetry_payload_t *application_payload) {
  /* Availability block: diagnostics-disabled or partial startup is a no-op. */
  if (telemetry_reporter != NULL) {
    (void)esp_rt_diag_reporter_publish(telemetry_reporter, diagnostics,
                                       application_payload);
  }
}
