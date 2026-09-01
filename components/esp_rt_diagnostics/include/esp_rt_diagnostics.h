#ifndef ESP_RT_DIAGNOSTICS_H_
#define ESP_RT_DIAGNOSTICS_H_

#include "esp_err.h"
#include "esp_timer.h"
#include "sdkconfig.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ESP_RT_DIAG_MAX_STAGES 8U
#define ESP_RT_DIAG_MAX_EVENTS 8U
#define ESP_RT_DIAG_MAX_INTERVALS 4U

typedef struct {
  uint32_t calls;
  uint32_t max_duration_us;
} esp_rt_diag_stage_snapshot_t;

typedef struct {
  uint32_t samples;
  uint32_t min_us;
  uint32_t max_us;
} esp_rt_diag_interval_snapshot_t;

/** Immutable diagnostic result for one measurement window. */
typedef struct {
  uint32_t expected_period_us;
  uint32_t deadline_us;
  uint32_t window_duration_us;
  uint32_t cycles;
  uint32_t cycles_with_valid_wake_time;
  uint32_t missed_events;
  uint32_t deadline_overruns;
  uint32_t max_wake_latency_us;
  uint32_t max_processing_time_us;
  uint32_t max_cycle_time_us;
  uint32_t lifetime_max_processing_time_us;
  uint32_t total_cycles;
  uint32_t total_missed_events;
  uint32_t total_deadline_overruns;
  uint8_t stage_count;
  uint8_t event_count;
  uint8_t interval_count;
  uint32_t event_counts[ESP_RT_DIAG_MAX_EVENTS];
  uint32_t total_event_counts[ESP_RT_DIAG_MAX_EVENTS];
  esp_rt_diag_stage_snapshot_t stages[ESP_RT_DIAG_MAX_STAGES];
  esp_rt_diag_interval_snapshot_t intervals[ESP_RT_DIAG_MAX_INTERVALS];
} esp_rt_diag_snapshot_t;

typedef struct {
  uint32_t expected_period_us;
  uint32_t deadline_us;
  uint8_t stage_count;
  uint8_t event_count;
  uint8_t interval_count;
} esp_rt_diag_config_t;

/**
 * Accumulator owned by exactly one monitored task.
 *
 * It intentionally contains no lock, queue, logger, or dynamically allocated
 * resource. A snapshot may only be taken by the same task that updates it.
 */
typedef struct {
  esp_rt_diag_config_t config;
  int64_t window_start_us;
  int64_t processing_start_us;
  uint32_t current_wake_latency_us;
  bool current_wake_time_valid;
  bool cycle_open;

  uint32_t cycles;
  uint32_t cycles_with_valid_wake_time;
  uint32_t missed_events;
  uint32_t deadline_overruns;
  uint32_t max_wake_latency_us;
  uint32_t max_processing_time_us;
  uint32_t max_cycle_time_us;
  uint32_t lifetime_max_processing_time_us;
  uint32_t total_cycles;
  uint32_t total_missed_events;
  uint32_t total_deadline_overruns;
  uint32_t event_counts[ESP_RT_DIAG_MAX_EVENTS];
  uint32_t total_event_counts[ESP_RT_DIAG_MAX_EVENTS];
  esp_rt_diag_stage_snapshot_t stages[ESP_RT_DIAG_MAX_STAGES];
  esp_rt_diag_interval_snapshot_t intervals[ESP_RT_DIAG_MAX_INTERVALS];
} esp_rt_diag_t;

esp_err_t esp_rt_diag_init(esp_rt_diag_t *diagnostics,
                           const esp_rt_diag_config_t *config,
                           int64_t start_time_us);

esp_err_t esp_rt_diag_take_snapshot(esp_rt_diag_t *diagnostics, int64_t now_us,
                                    esp_rt_diag_snapshot_t *snapshot);

static inline void esp_rt_diag_update_max(uint32_t *maximum, uint32_t value) {
  if (value > *maximum) {
    *maximum = value;
  }
}

/** Timestamp helper compiled out together with detailed timing. */
static inline int64_t esp_rt_diag_stage_begin(void) {
#if CONFIG_ESP_RT_DIAGNOSTICS_ENABLE &&                                        \
    CONFIG_ESP_RT_DIAGNOSTICS_DETAILED_TIMING
  return esp_timer_get_time();
#else
  return 0;
#endif
}

/** Capture an ISR timestamp; the volatile write is removed when disabled. */
static inline void esp_rt_diag_isr_capture(volatile uint32_t *destination) {
#if CONFIG_ESP_RT_DIAGNOSTICS_ENABLE
  if (destination != NULL) {
    *destination = (uint32_t)esp_timer_get_time();
  }
#else
  (void)destination;
#endif
}

/** Begin one task cycle after it wakes from its periodic event. */
static inline void esp_rt_diag_cycle_begin(esp_rt_diag_t *diagnostics,
                                           uint32_t event_time_us,
                                           uint32_t pending_events,
                                           int64_t wake_time_us) {
#if CONFIG_ESP_RT_DIAGNOSTICS_ENABLE
  if (diagnostics == NULL) {
    return;
  }

  diagnostics->processing_start_us = wake_time_us;
  diagnostics->cycle_open = true;
  diagnostics->current_wake_time_valid = pending_events == 1U;
  diagnostics->current_wake_latency_us = 0;

  if (pending_events > 1U) {
    uint32_t missed = pending_events - 1U;
    diagnostics->missed_events += missed;
    diagnostics->total_missed_events += missed;
  } else if (pending_events == 1U) {
    uint32_t wake_latency_us = (uint32_t)wake_time_us - event_time_us;
    diagnostics->current_wake_latency_us = wake_latency_us;
    diagnostics->cycles_with_valid_wake_time++;
    esp_rt_diag_update_max(&diagnostics->max_wake_latency_us, wake_latency_us);
  }
#else
  (void)diagnostics;
  (void)event_time_us;
  (void)pending_events;
  (void)wake_time_us;
#endif
}

/** Begin a cycle using the component clock. */
static inline void esp_rt_diag_cycle_begin_now(esp_rt_diag_t *diagnostics,
                                               uint32_t event_time_us,
                                               uint32_t pending_events) {
#if CONFIG_ESP_RT_DIAGNOSTICS_ENABLE
  esp_rt_diag_cycle_begin(diagnostics, event_time_us, pending_events,
                          esp_timer_get_time());
#else
  (void)diagnostics;
  (void)event_time_us;
  (void)pending_events;
#endif
}

/** Begin a cycle from the timestamp shared by its ISR. */
static inline void
esp_rt_diag_cycle_begin_from_isr(esp_rt_diag_t *diagnostics,
                                 const volatile uint32_t *event_time_us,
                                 uint32_t pending_events) {
#if CONFIG_ESP_RT_DIAGNOSTICS_ENABLE
  if (event_time_us != NULL) {
    esp_rt_diag_cycle_begin_now(diagnostics, *event_time_us, pending_events);
  }
#else
  (void)diagnostics;
  (void)event_time_us;
  (void)pending_events;
#endif
}

/** Finish the current cycle and evaluate its configured deadline. */
static inline void esp_rt_diag_cycle_end(esp_rt_diag_t *diagnostics,
                                         int64_t end_time_us) {
#if CONFIG_ESP_RT_DIAGNOSTICS_ENABLE
  if (diagnostics == NULL || !diagnostics->cycle_open) {
    return;
  }

  uint32_t processing_us =
      (uint32_t)(end_time_us - diagnostics->processing_start_us);
  esp_rt_diag_update_max(&diagnostics->max_processing_time_us, processing_us);
  esp_rt_diag_update_max(&diagnostics->lifetime_max_processing_time_us,
                         processing_us);

  uint32_t deadline_consumption_us = processing_us;
  if (diagnostics->current_wake_time_valid) {
    deadline_consumption_us += diagnostics->current_wake_latency_us;
    esp_rt_diag_update_max(&diagnostics->max_cycle_time_us,
                           deadline_consumption_us);
  }

  if (deadline_consumption_us > diagnostics->config.deadline_us) {
    diagnostics->deadline_overruns++;
    diagnostics->total_deadline_overruns++;
  }
  diagnostics->cycles++;
  diagnostics->total_cycles++;
  diagnostics->cycle_open = false;
#else
  (void)diagnostics;
  (void)end_time_us;
#endif
}

/** Finish a cycle and return its end timestamp for snapshot scheduling. */
static inline int64_t esp_rt_diag_cycle_end_now(esp_rt_diag_t *diagnostics) {
#if CONFIG_ESP_RT_DIAGNOSTICS_ENABLE
  int64_t end_time_us = esp_timer_get_time();
  esp_rt_diag_cycle_end(diagnostics, end_time_us);
  return end_time_us;
#else
  (void)diagnostics;
  return 0;
#endif
}

/** Test the configured window without an out-of-line hot-path call. */
static inline bool esp_rt_diag_snapshot_due(const esp_rt_diag_t *diagnostics,
                                            int64_t now_us) {
#if CONFIG_ESP_RT_DIAGNOSTICS_ENABLE
  if (diagnostics == NULL) {
    return false;
  }
  const int64_t window_us =
      (int64_t)CONFIG_ESP_RT_DIAGNOSTICS_WINDOW_MS * 1000LL;
  return now_us - diagnostics->window_start_us >= window_us;
#else
  (void)diagnostics;
  (void)now_us;
  return false;
#endif
}

/** Count a successful operation or an application-defined error/event. */
static inline void esp_rt_diag_event(esp_rt_diag_t *diagnostics,
                                     uint8_t event_id, uint32_t amount) {
#if CONFIG_ESP_RT_DIAGNOSTICS_ENABLE
  if (diagnostics != NULL && event_id < diagnostics->config.event_count) {
    diagnostics->event_counts[event_id] += amount;
    diagnostics->total_event_counts[event_id] += amount;
  }
#else
  (void)diagnostics;
  (void)event_id;
  (void)amount;
#endif
}

/** Record a named stage from timestamps already available to the application.
 */
static inline void esp_rt_diag_stage_record(esp_rt_diag_t *diagnostics,
                                            uint8_t stage_id,
                                            int64_t start_time_us,
                                            int64_t end_time_us) {
#if CONFIG_ESP_RT_DIAGNOSTICS_ENABLE &&                                        \
    CONFIG_ESP_RT_DIAGNOSTICS_DETAILED_TIMING
  if (diagnostics != NULL && stage_id < diagnostics->config.stage_count) {
    uint32_t duration_us = (uint32_t)(end_time_us - start_time_us);
    diagnostics->stages[stage_id].calls++;
    esp_rt_diag_update_max(&diagnostics->stages[stage_id].max_duration_us,
                           duration_us);
  }
#else
  (void)diagnostics;
  (void)stage_id;
  (void)start_time_us;
  (void)end_time_us;
#endif
}

/** Record a completed named stage using the component clock. */
static inline void esp_rt_diag_stage_end(esp_rt_diag_t *diagnostics,
                                         uint8_t stage_id,
                                         int64_t start_time_us) {
#if CONFIG_ESP_RT_DIAGNOSTICS_ENABLE &&                                        \
    CONFIG_ESP_RT_DIAGNOSTICS_DETAILED_TIMING
  esp_rt_diag_stage_record(diagnostics, stage_id, start_time_us,
                           esp_timer_get_time());
#else
  (void)diagnostics;
  (void)stage_id;
  (void)start_time_us;
#endif
}

/** Record an already measured periodic interval. */
static inline void esp_rt_diag_interval(esp_rt_diag_t *diagnostics,
                                        uint8_t interval_id,
                                        uint32_t interval_us) {
#if CONFIG_ESP_RT_DIAGNOSTICS_ENABLE &&                                        \
    CONFIG_ESP_RT_DIAGNOSTICS_DETAILED_TIMING
  if (diagnostics != NULL && interval_id < diagnostics->config.interval_count) {
    esp_rt_diag_interval_snapshot_t *interval =
        &diagnostics->intervals[interval_id];
    if (interval->samples == 0U || interval_us < interval->min_us) {
      interval->min_us = interval_us;
    }
    if (interval_us > interval->max_us) {
      interval->max_us = interval_us;
    }
    interval->samples++;
  }
#else
  (void)diagnostics;
  (void)interval_id;
  (void)interval_us;
#endif
}

#ifdef __cplusplus
}
#endif

#endif /* ESP_RT_DIAGNOSTICS_H_ */
