#include "esp_rt_diagnostics.h"

#include <string.h>

static void reset_window(esp_rt_diag_t *diagnostics, int64_t start_time_us) {
  diagnostics->window_start_us = start_time_us;
  diagnostics->cycles = 0;
  diagnostics->cycles_with_valid_wake_time = 0;
  diagnostics->missed_events = 0;
  diagnostics->deadline_overruns = 0;
  diagnostics->max_wake_latency_us = 0;
  diagnostics->max_processing_time_us = 0;
  diagnostics->max_cycle_time_us = 0;
  memset(diagnostics->event_counts, 0, sizeof(diagnostics->event_counts));
  memset(diagnostics->stages, 0, sizeof(diagnostics->stages));
  memset(diagnostics->intervals, 0, sizeof(diagnostics->intervals));
}

esp_err_t esp_rt_diag_init(esp_rt_diag_t *diagnostics,
                           const esp_rt_diag_config_t *config,
                           int64_t start_time_us) {
  if (diagnostics == NULL || config == NULL ||
      config->expected_period_us == 0U || config->deadline_us == 0U ||
      config->stage_count > ESP_RT_DIAG_MAX_STAGES ||
      config->event_count > ESP_RT_DIAG_MAX_EVENTS ||
      config->interval_count > ESP_RT_DIAG_MAX_INTERVALS) {
    return ESP_ERR_INVALID_ARG;
  }

  memset(diagnostics, 0, sizeof(*diagnostics));
  diagnostics->config = *config;
  reset_window(diagnostics, start_time_us);
  return ESP_OK;
}

esp_err_t esp_rt_diag_take_snapshot(esp_rt_diag_t *diagnostics, int64_t now_us,
                                    esp_rt_diag_snapshot_t *snapshot) {
  if (diagnostics == NULL || snapshot == NULL ||
      now_us < diagnostics->window_start_us || diagnostics->cycle_open) {
    return ESP_ERR_INVALID_ARG;
  }

  *snapshot = (esp_rt_diag_snapshot_t){
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
  memcpy(snapshot->event_counts, diagnostics->event_counts,
         sizeof(snapshot->event_counts));
  memcpy(snapshot->total_event_counts, diagnostics->total_event_counts,
         sizeof(snapshot->total_event_counts));
  memcpy(snapshot->stages, diagnostics->stages, sizeof(snapshot->stages));
  memcpy(snapshot->intervals, diagnostics->intervals,
         sizeof(snapshot->intervals));

  reset_window(diagnostics, now_us);
  return ESP_OK;
}
