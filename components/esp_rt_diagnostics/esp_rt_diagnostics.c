#include "esp_rt_diagnostics.h"

#include <string.h>

#ifdef CONFIG_ESP_RT_DIAGNOSTICS_WINDOW_MS
#define ESP_RT_DIAG_DEFAULT_WINDOW_US                                          \
  ((uint32_t)CONFIG_ESP_RT_DIAGNOSTICS_WINDOW_MS * 1000U)
#else
/* Kconfig omits the dependent option when instrumentation is disabled. */
#define ESP_RT_DIAG_DEFAULT_WINDOW_US 5000000U
#endif

/**
 * Clear only the active-window accumulators and move its time origin.
 *
 * Internal helper called by esp_rt_diag_init() and
 * esp_rt_diag_take_snapshot(). Lifetime counters, configuration, and window_id
 * deliberately survive a reset performed after a snapshot.
 */
static void reset_window(esp_rt_diag_t *diagnostics, int64_t start_time_us) {
  /* Establish the new window boundary and clear scalar window statistics. */
  diagnostics->window_start_us = start_time_us;
  diagnostics->cycles = 0;
  diagnostics->cycles_with_valid_wake_time = 0;
  diagnostics->missed_events = 0;
  diagnostics->deadline_overruns = 0;
  diagnostics->max_wake_latency_us = 0;
  diagnostics->max_processing_time_us = 0;
  diagnostics->max_cycle_time_us = 0;
  /* Clear every fixed-capacity array, including currently unused entries. */
  memset(diagnostics->event_counts, 0, sizeof(diagnostics->event_counts));
  memset(diagnostics->stages, 0, sizeof(diagnostics->stages));
  memset(diagnostics->intervals, 0, sizeof(diagnostics->intervals));
}

/**
 * Initialize a diagnostic accumulator.
 *
 * Public entry point, not called internally. See the public header for the
 * complete contract.
 */
esp_err_t esp_rt_diag_init(esp_rt_diag_t *diagnostics,
                           const esp_rt_diag_config_t *config,
                           int64_t start_time_us) {
  /* Reject inputs that would make IDs or deadline calculations invalid. */
  if (diagnostics == NULL || config == NULL ||
      config->expected_period_us == 0U || config->deadline_us == 0U ||
      config->stage_count > ESP_RT_DIAG_MAX_STAGES ||
      config->event_count > ESP_RT_DIAG_MAX_EVENTS ||
      config->interval_count > ESP_RT_DIAG_MAX_INTERVALS) {
    return ESP_ERR_INVALID_ARG;
  }

  /* Reinitialization intentionally discards both window and lifetime history.
   */
  memset(diagnostics, 0, sizeof(*diagnostics));
  diagnostics->config = *config;

  /* Resolve the optional per-instance override before opening window 1. */
  if (diagnostics->config.window_duration_us == 0U) {
    diagnostics->config.window_duration_us = ESP_RT_DIAG_DEFAULT_WINDOW_US;
  }
  diagnostics->window_id = 1U;
  reset_window(diagnostics, start_time_us);
  return ESP_OK;
}

/**
 * Close the active window, copy its results, and open the next window.
 *
 * Public entry point called directly by applications and internally by
 * esp_rt_diag_try_take_snapshot().
 */
esp_err_t esp_rt_diag_take_snapshot(esp_rt_diag_t *diagnostics, int64_t now_us,
                                    esp_rt_diag_snapshot_t *snapshot) {
  /* A snapshot boundary may not split an open cycle or move time backward. */
  if (diagnostics == NULL || snapshot == NULL ||
      now_us < diagnostics->window_start_us || diagnostics->cycle_open) {
    return ESP_ERR_INVALID_ARG;
  }

  /* Copy scalar configuration, window statistics, and lifetime statistics. */
  *snapshot = (esp_rt_diag_snapshot_t){
      .window_id = diagnostics->window_id,
      .expected_period_us = diagnostics->config.expected_period_us,
      .deadline_us = diagnostics->config.deadline_us,
      .window_duration_us = (uint32_t)(now_us - diagnostics->window_start_us),
      .cycles = diagnostics->cycles,
      .cycles_with_valid_wake_time = diagnostics->cycles_with_valid_wake_time,
      .missed_events = diagnostics->missed_events,
      .deadline_overruns = diagnostics->deadline_overruns,
      .max_wake_latency_us = diagnostics->max_wake_latency_us,
      .max_processing_time_us = diagnostics->max_processing_time_us,
      .max_cycle_time_us = diagnostics->max_cycle_time_us,
      .lifetime_max_processing_time_us =
          diagnostics->lifetime_max_processing_time_us,
      .total_cycles = diagnostics->total_cycles,
      .total_missed_events = diagnostics->total_missed_events,
      .total_deadline_overruns = diagnostics->total_deadline_overruns,
      .stage_count = diagnostics->config.stage_count,
      .event_count = diagnostics->config.event_count,
      .interval_count = diagnostics->config.interval_count,
  };
  /* Copy fixed-capacity arrays so the result no longer aliases live state. */
  memcpy(snapshot->event_counts, diagnostics->event_counts,
         sizeof(snapshot->event_counts));
  memcpy(snapshot->total_event_counts, diagnostics->total_event_counts,
         sizeof(snapshot->total_event_counts));
  memcpy(snapshot->stages, diagnostics->stages, sizeof(snapshot->stages));
  memcpy(snapshot->intervals, diagnostics->intervals,
         sizeof(snapshot->intervals));

  /* Preserve lifetime state while advancing to the next measurement window. */
  diagnostics->window_id++;
  reset_window(diagnostics, now_us);
  return ESP_OK;
}

/**
 * Conditionally close a window once its configured duration has elapsed.
 *
 * Public entry point, not called internally. Calls
 * esp_rt_diag_snapshot_due() and esp_rt_diag_take_snapshot().
 */
esp_err_t esp_rt_diag_try_take_snapshot(esp_rt_diag_t *diagnostics,
                                        int64_t now_us,
                                        esp_rt_diag_snapshot_t *snapshot,
                                        bool *snapshot_taken) {
  /* Make the success-without-output state explicit to the caller. */
  if (snapshot_taken == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  *snapshot_taken = false;

  /* Validate state even before the due test to keep misuse observable. */
  if (diagnostics == NULL || snapshot == NULL ||
      now_us < diagnostics->window_start_us || diagnostics->cycle_open) {
    return ESP_ERR_INVALID_ARG;
  }

  /* Not due is normal flow, not an error and not a mutation. */
  if (!esp_rt_diag_snapshot_due(diagnostics, now_us)) {
    return ESP_OK;
  }

  /* Delegate copying/reset semantics to the unconditional operation. */
  esp_err_t error = esp_rt_diag_take_snapshot(diagnostics, now_us, snapshot);
  if (error == ESP_OK) {
    *snapshot_taken = true;
  }
  return error;
}

/**
 * Calculate one event's average frequency over the actual snapshot duration.
 *
 * Public helper called by the reporter's full and compact formats. Invalid or
 * empty inputs return zero so presentation code never divides by zero.
 */
float esp_rt_diag_event_rate_hz(const esp_rt_diag_snapshot_t *snapshot,
                                uint8_t event_id) {
  if (snapshot == NULL || snapshot->window_duration_us == 0U ||
      event_id >= snapshot->event_count) {
    return 0.0f;
  }
  /* Divide by actual elapsed time, not by the requested window duration. */
  return (float)snapshot->event_counts[event_id] * 1000000.0f /
         (float)snapshot->window_duration_us;
}

/**
 * Calculate the average completed-cycle frequency for one snapshot.
 *
 * Public helper called by the reporter's full and compact formats. Invalid or
 * zero-duration inputs return zero.
 */
float esp_rt_diag_cycle_rate_hz(const esp_rt_diag_snapshot_t *snapshot) {
  if (snapshot == NULL || snapshot->window_duration_us == 0U) {
    return 0.0f;
  }
  return (float)snapshot->cycles * 1000000.0f /
         (float)snapshot->window_duration_us;
}

/**
 * Calculate deadline overruns as a percentage of completed cycles.
 *
 * Public helper called by the reporter's full and compact formats. A NULL or
 * cycle-free snapshot returns zero.
 */
float esp_rt_diag_deadline_overrun_percent(
    const esp_rt_diag_snapshot_t *snapshot) {
  if (snapshot == NULL || snapshot->cycles == 0U) {
    return 0.0f;
  }
  return 100.0f * (float)snapshot->deadline_overruns / (float)snapshot->cycles;
}

/**
 * Calculate one stage's arithmetic mean execution time in microseconds.
 *
 * Public helper called by the reporter's full and compact formats. Invalid IDs
 * and stages with no recorded calls return zero.
 */
float esp_rt_diag_stage_average_us(const esp_rt_diag_snapshot_t *snapshot,
                                   uint8_t stage_id) {
  if (snapshot == NULL || stage_id >= snapshot->stage_count ||
      snapshot->stages[stage_id].calls == 0U) {
    return 0.0f;
  }
  /* Delay conversion to float until outside the real-time accumulation path. */
  return (float)snapshot->stages[stage_id].total_duration_us /
         (float)snapshot->stages[stage_id].calls;
}
