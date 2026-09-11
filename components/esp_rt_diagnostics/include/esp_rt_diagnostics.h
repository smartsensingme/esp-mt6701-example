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

#define ESP_RT_DIAG_MAX_STAGES 8U /**< Maximum named stages per instance. */
#define ESP_RT_DIAG_MAX_EVENTS 8U /**< Maximum event counters per instance. */
#define ESP_RT_DIAG_MAX_INTERVALS 4U /**< Maximum intervals per instance. */

/** Statistics accumulated for one instrumented stage during one window. */
typedef struct {
  uint32_t calls;           /**< Number of recorded executions in the window. */
  uint32_t max_duration_us; /**< Longest recorded execution, in microseconds. */
  uint64_t
      total_duration_us; /**< Sum used to derive the mean, in microseconds. */
  uint32_t
      over_budget_calls; /**< Executions strictly longer than their budget. */
} esp_rt_diag_stage_snapshot_t;

/** Minimum and maximum values of one application-defined time interval. */
typedef struct {
  uint32_t samples; /**< Number of intervals recorded in the window. */
  uint32_t min_us;  /**< Smallest recorded interval, in microseconds. */
  uint32_t max_us;  /**< Largest recorded interval, in microseconds. */
} esp_rt_diag_interval_snapshot_t;

/**
 * Immutable diagnostic result for one completed measurement window.
 *
 * The owning task creates this value with esp_rt_diag_take_snapshot() or
 * esp_rt_diag_try_take_snapshot(). After creation it is independent of the
 * live accumulator and may be copied to another task.
 */
typedef struct {
  uint64_t
      window_id; /**< Monotonic instance-local identifier, starting at 1. */
  uint32_t expected_period_us; /**< Configured cycle period, in microseconds. */
  uint32_t deadline_us; /**< Configured cycle deadline, in microseconds. */
  uint32_t
      window_duration_us; /**< Actual elapsed window time, in microseconds. */
  uint32_t cycles;        /**< Completed cycles in this window. */
  uint32_t
      cycles_with_valid_wake_time; /**< Cycles with unambiguous ISR latency. */
  uint32_t missed_events; /**< Excess pending periodic events in this window. */
  uint32_t deadline_overruns; /**< Cycles exceeding deadline in this window. */
  uint32_t max_wake_latency_us; /**< Maximum valid ISR-to-task latency. */
  uint32_t
      max_processing_time_us; /**< Maximum task processing time in window. */
  uint32_t max_cycle_time_us; /**< Maximum valid wake plus processing time. */
  uint32_t
      lifetime_max_processing_time_us; /**< Maximum since initialization. */
  uint32_t total_cycles;        /**< Completed cycles since initialization. */
  uint32_t total_missed_events; /**< Missed events since initialization. */
  uint32_t
      total_deadline_overruns; /**< Deadline overruns since initialization. */
  uint8_t stage_count;         /**< Valid entries in stages[]. */
  uint8_t event_count;         /**< Valid entries in event_counts[] arrays. */
  uint8_t interval_count;      /**< Valid entries in intervals[]. */
  uint32_t
      event_counts[ESP_RT_DIAG_MAX_EVENTS]; /**< Per-window event totals. */
  uint32_t
      total_event_counts[ESP_RT_DIAG_MAX_EVENTS]; /**< Lifetime event totals. */
  esp_rt_diag_stage_snapshot_t
      stages[ESP_RT_DIAG_MAX_STAGES]; /**< Stage results. */
  esp_rt_diag_interval_snapshot_t intervals[ESP_RT_DIAG_MAX_INTERVALS];
} esp_rt_diag_snapshot_t;

/** Configuration copied into one esp_rt_diag_t by esp_rt_diag_init(). */
typedef struct {
  uint32_t expected_period_us; /**< Nominal event period; must be nonzero. */
  uint32_t
      deadline_us; /**< Maximum allowed wake plus processing time; nonzero. */
  uint32_t
      window_duration_us; /**< Window length; zero selects Kconfig default. */
  uint8_t stage_count;    /**< Used stage IDs: 0 to ESP_RT_DIAG_MAX_STAGES. */
  uint8_t event_count;    /**< Used event IDs: 0 to ESP_RT_DIAG_MAX_EVENTS. */
  uint8_t
      interval_count; /**< Used interval IDs: 0 to ESP_RT_DIAG_MAX_INTERVALS. */
  /** Per-stage limits in microseconds; zero disables violation counting. */
  uint32_t stage_budget_us[ESP_RT_DIAG_MAX_STAGES];
} esp_rt_diag_config_t;

/**
 * Accumulator owned by exactly one monitored task.
 *
 * It intentionally contains no lock, queue, logger, or dynamically allocated
 * resource. A snapshot may only be taken by the same task that updates it.
 */
typedef struct {
  esp_rt_diag_config_t
      config;              /**< Private working copy of the configuration. */
  uint64_t window_id;      /**< Identifier assigned to the active window. */
  int64_t window_start_us; /**< Active-window start on the esp_timer clock. */
  int64_t processing_start_us; /**< Start time of the currently open cycle. */
  uint32_t current_wake_latency_us; /**< Valid latency for the open cycle. */
  bool current_wake_time_valid; /**< Whether current wake latency is usable. */
  bool cycle_open;              /**< True between cycle_begin and cycle_end. */

  uint32_t cycles;                      /**< Window cycle counter. */
  uint32_t cycles_with_valid_wake_time; /**< Window valid-wake counter. */
  uint32_t missed_events;               /**< Window missed-event counter. */
  uint32_t deadline_overruns;           /**< Window deadline-overrun counter. */
  uint32_t max_wake_latency_us;         /**< Window maximum wake latency. */
  uint32_t max_processing_time_us;      /**< Window maximum processing time. */
  uint32_t max_cycle_time_us;           /**< Window maximum valid cycle time. */
  uint32_t lifetime_max_processing_time_us; /**< Lifetime processing maximum. */
  uint32_t total_cycles;                    /**< Lifetime cycle counter. */
  uint32_t total_missed_events;     /**< Lifetime missed-event counter. */
  uint32_t total_deadline_overruns; /**< Lifetime deadline-overrun counter. */
  uint32_t event_counts[ESP_RT_DIAG_MAX_EVENTS]; /**< Window event counters. */
  uint32_t total_event_counts[ESP_RT_DIAG_MAX_EVENTS]; /**< Lifetime events. */
  esp_rt_diag_stage_snapshot_t
      stages[ESP_RT_DIAG_MAX_STAGES]; /**< Window stages. */
  esp_rt_diag_interval_snapshot_t intervals[ESP_RT_DIAG_MAX_INTERVALS];
} esp_rt_diag_t;

/**
 * Initialize or reinitialize a task-owned diagnostic accumulator.
 *
 * External API; not called internally. Copies config, resolves a zero window
 * duration from Kconfig, clears both window and lifetime state, and opens
 * window 1 at start_time_us.
 *
 * @param diagnostics Writable storage that remains valid while it is used.
 * @param config Configuration copied by value; the source may then disappear.
 * @param start_time_us Window origin from esp_timer_get_time(), in
 * microseconds.
 * @return ESP_OK, or ESP_ERR_INVALID_ARG for NULL pointers, zero
 * period/deadline, or a configured count above its compile-time maximum.
 * @note Call from task context before any inline recording function. The
 *       instance is single-owner and provides no synchronization.
 */
esp_err_t esp_rt_diag_init(esp_rt_diag_t *diagnostics,
                           const esp_rt_diag_config_t *config,
                           int64_t start_time_us);

/**
 * Materialize the active window and immediately begin the next one.
 *
 * External API; also called by esp_rt_diag_try_take_snapshot(). Copies all
 * window and lifetime values, increments window_id, then resets only the
 * per-window accumulators.
 *
 * @param diagnostics Initialized accumulator owned by the calling task.
 * @param now_us End of the window, on the same clock as start_time_us.
 * @param snapshot Destination receiving a complete value-owned copy.
 * @return ESP_OK, or ESP_ERR_INVALID_ARG if a pointer is NULL, time moves
 *         backwards, or a cycle is still open.
 */
esp_err_t esp_rt_diag_take_snapshot(esp_rt_diag_t *diagnostics, int64_t now_us,
                                    esp_rt_diag_snapshot_t *snapshot);

/**
 * Take a snapshot only when the configured per-instance window is complete.
 *
 * External API; calls esp_rt_diag_snapshot_due() and, when due,
 * esp_rt_diag_take_snapshot(). Not-due is a successful result with
 * snapshot_taken set to false.
 *
 * @param diagnostics Initialized accumulator owned by the calling task.
 * @param now_us Candidate window end, in microseconds on the component clock.
 * @param snapshot Destination written only when a snapshot is taken.
 * @param snapshot_taken Mandatory result flag, initialized to false by this
 * call.
 * @return ESP_OK for both taken and not-yet-due states; ESP_ERR_INVALID_ARG for
 *         invalid pointers, backward time, or an open cycle.
 */
esp_err_t esp_rt_diag_try_take_snapshot(esp_rt_diag_t *diagnostics,
                                        int64_t now_us,
                                        esp_rt_diag_snapshot_t *snapshot,
                                        bool *snapshot_taken);

/**
 * Calculate one event rate over the snapshot's actual duration.
 *
 * External utility; called by both generic reporter formats.
 * @param snapshot Completed immutable snapshot.
 * @param event_id Zero-based ID smaller than snapshot->event_count.
 * @return Events per second, or 0 for NULL, zero duration, or invalid event_id.
 */
float esp_rt_diag_event_rate_hz(const esp_rt_diag_snapshot_t *snapshot,
                                uint8_t event_id);

/**
 * Calculate the completed-cycle rate over the actual window duration.
 * External utility; called by both generic reporter formats.
 * @param snapshot Completed immutable snapshot.
 * @return Cycles per second, or 0 for NULL or zero-duration input.
 */
float esp_rt_diag_cycle_rate_hz(const esp_rt_diag_snapshot_t *snapshot);

/**
 * Calculate the percentage of completed cycles that exceeded the deadline.
 * External utility; called by both generic reporter formats.
 * @param snapshot Completed immutable snapshot.
 * @return Percentage in [0,100] for consistent counters, or 0 for no cycles.
 */
float esp_rt_diag_deadline_overrun_percent(
    const esp_rt_diag_snapshot_t *snapshot);

/**
 * Calculate the arithmetic mean duration of one instrumented stage.
 * External utility; called by both generic reporter formats.
 * @param snapshot Completed immutable snapshot.
 * @param stage_id Zero-based ID smaller than snapshot->stage_count.
 * @return Microseconds, or 0 for NULL, invalid stage_id, or no stage calls.
 */
float esp_rt_diag_stage_average_us(const esp_rt_diag_snapshot_t *snapshot,
                                   uint8_t stage_id);

/**
 * Replace a running maximum when value is greater.
 *
 * Internal inline helper called by esp_rt_diag_cycle_begin(),
 * esp_rt_diag_cycle_end(), and esp_rt_diag_stage_record(). It is visible in the
 * public header only so those hot-path functions can remain inline.
 * @param maximum Non-NULL running maximum to update.
 * @param value Candidate unsigned value.
 */
static inline void esp_rt_diag_update_max(uint32_t *maximum, uint32_t value) {
  /* A strict comparison preserves the existing maximum on equal samples. */
  if (value > *maximum) {
    *maximum = value;
  }
}

/**
 * Read the start timestamp for a detailed stage measurement.
 *
 * External hot-path helper; normally paired with esp_rt_diag_stage_end() or
 * esp_rt_diag_stage_record(). It returns esp_timer microseconds when both
 * diagnostics and detailed timing are enabled, otherwise zero.
 * @return Monotonic timestamp in microseconds, or zero when compiled out.
 */
static inline int64_t esp_rt_diag_stage_begin(void) {
#if CONFIG_ESP_RT_DIAGNOSTICS_ENABLE &&                                        \
    CONFIG_ESP_RT_DIAGNOSTICS_DETAILED_TIMING
  return esp_timer_get_time();
#else
  return 0;
#endif
}

/**
 * Capture the most recent periodic-event timestamp from ISR context.
 *
 * External ISR helper; the task later passes the same volatile storage to
 * esp_rt_diag_cycle_begin_from_isr(). The destination must remain valid and be
 * readable atomically as uint32_t. The low 32 bits wrap about every 71.6 min;
 * unsigned subtraction remains correct when latency spans less than one wrap.
 * @param destination Persistent volatile uint32_t shared with the task; NULL is
 *        accepted as a no-op.
 */
static inline void esp_rt_diag_isr_capture(volatile uint32_t *destination) {
#if CONFIG_ESP_RT_DIAGNOSTICS_ENABLE
  /* Keep ISR work bounded to one timestamp read and one volatile store. */
  if (destination != NULL) {
    *destination = (uint32_t)esp_timer_get_time();
  }
#else
  (void)destination;
#endif
}

/**
 * Open one monitored task cycle using an already-read wake timestamp.
 *
 * External low-level API; called internally by esp_rt_diag_cycle_begin_now().
 * Exactly one pending event produces a valid wake latency. More than one means
 * pending_events - 1 deadlines were missed and the remaining timestamp is
 * ambiguous; zero opens the processing measurement without wake timing.
 * Call esp_rt_diag_cycle_end() before beginning another cycle or snapshotting.
 * @param diagnostics Initialized task-owned accumulator; NULL is a no-op.
 * @param event_time_us Low 32 bits of the relevant ISR timestamp.
 * @param pending_events Number of periodic notifications consumed by this wake.
 * @param wake_time_us Task wake timestamp in monotonic microseconds.
 */
static inline void esp_rt_diag_cycle_begin(esp_rt_diag_t *diagnostics,
                                           uint32_t event_time_us,
                                           uint32_t pending_events,
                                           int64_t wake_time_us) {
#if CONFIG_ESP_RT_DIAGNOSTICS_ENABLE
  if (diagnostics == NULL) {
    return;
  }

  /* Open the processing interval even when no valid ISR timestamp exists. */
  diagnostics->processing_start_us = wake_time_us;
  diagnostics->cycle_open = true;
  diagnostics->current_wake_time_valid = pending_events == 1U;
  diagnostics->current_wake_latency_us = 0;

  /* Classify the notification count before updating wake statistics. */
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

/**
 * Open one monitored cycle and obtain the task wake time internally.
 *
 * External convenience API; called internally by
 * esp_rt_diag_cycle_begin_from_isr(). Calls esp_rt_diag_cycle_begin() after
 * reading esp_timer_get_time().
 * @param diagnostics Initialized task-owned accumulator; NULL is a no-op.
 * @param event_time_us Low 32 bits captured for the periodic ISR event.
 * @param pending_events Number of periodic notifications consumed by this wake.
 */
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

/**
 * Open one monitored cycle from volatile timestamp storage shared with an ISR.
 *
 * External convenience API. Reads the latest timestamp once and calls
 * esp_rt_diag_cycle_begin_now(). A NULL timestamp pointer is a no-op.
 * This function runs in task context; esp_rt_diag_isr_capture() is the ISR
 * half.
 * @param diagnostics Initialized task-owned accumulator; NULL is a no-op.
 * @param event_time_us Persistent volatile location written by the event ISR.
 * @param pending_events Number of notifications returned to the task.
 */
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

/**
 * Close the active task cycle and evaluate its configured deadline.
 *
 * External low-level API; called internally by esp_rt_diag_cycle_end_now().
 * Processing time is always recorded. Wake latency is added to cycle time only
 * when exactly one event was pending. A missing or already-closed cycle is a
 * no-op. end_time_us must use the same monotonic microsecond clock.
 * @param diagnostics Initialized task-owned accumulator; NULL is a no-op.
 * @param end_time_us Cycle completion timestamp, in microseconds.
 */
static inline void esp_rt_diag_cycle_end(esp_rt_diag_t *diagnostics,
                                         int64_t end_time_us) {
#if CONFIG_ESP_RT_DIAGNOSTICS_ENABLE
  if (diagnostics == NULL || !diagnostics->cycle_open) {
    return;
  }

  /* Measure execution owned by the task and update window/lifetime maxima. */
  uint32_t processing_us =
      (uint32_t)(end_time_us - diagnostics->processing_start_us);
  esp_rt_diag_update_max(&diagnostics->max_processing_time_us, processing_us);
  esp_rt_diag_update_max(&diagnostics->lifetime_max_processing_time_us,
                         processing_us);

  /* Include scheduler wake latency only when its event timestamp is unique. */
  uint32_t deadline_consumption_us = processing_us;
  if (diagnostics->current_wake_time_valid) {
    deadline_consumption_us += diagnostics->current_wake_latency_us;
    esp_rt_diag_update_max(&diagnostics->max_cycle_time_us,
                           deadline_consumption_us);
  }

  /* Finish accounting and close the state transition begun by cycle_begin. */
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

/**
 * Close the active cycle using the component clock.
 *
 * External convenience API. Calls esp_rt_diag_cycle_end() and returns the same
 * timestamp so the caller can test snapshot scheduling without another timer
 * read. Returns zero when diagnostics are compiled out.
 * @param diagnostics Initialized task-owned accumulator; NULL is a no-op.
 * @return Cycle-end timestamp in microseconds, or zero when compiled out.
 */
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

/**
 * Test whether the active measurement window has reached its duration.
 *
 * External hot-path predicate; called by esp_rt_diag_try_take_snapshot(). It
 * never mutates state. Returns false for NULL or when diagnostics are disabled.
 * @param diagnostics Initialized accumulator to inspect.
 * @param now_us Candidate current timestamp in monotonic microseconds.
 * @return True when elapsed time is at least window_duration_us.
 */
static inline bool esp_rt_diag_snapshot_due(const esp_rt_diag_t *diagnostics,
                                            int64_t now_us) {
#if CONFIG_ESP_RT_DIAGNOSTICS_ENABLE
  if (diagnostics == NULL) {
    return false;
  }
  return now_us - diagnostics->window_start_us >=
         (int64_t)diagnostics->config.window_duration_us;
#else
  (void)diagnostics;
  (void)now_us;
  return false;
#endif
}

/**
 * Add an arbitrary amount to one application-defined event counter.
 *
 * External hot-path API; not called internally. Updates both the current-window
 * and lifetime counters. Invalid instances/IDs are ignored; arithmetic is
 * unsigned and intentionally has no saturation.
 * @param diagnostics Initialized task-owned accumulator; NULL is a no-op.
 * @param event_id Zero-based ID smaller than configured event_count.
 * @param amount Unsigned increment, commonly 1.
 */
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

/**
 * Record a named stage from timestamps already available to the application.
 *
 * External low-level API; called internally by esp_rt_diag_stage_end(). Updates
 * calls, total, maximum, and an optional strict budget-violation counter. It is
 * a no-op unless detailed timing is enabled. Times are monotonic microseconds.
 * @param diagnostics Initialized task-owned accumulator; NULL is a no-op.
 * @param stage_id Zero-based ID smaller than configured stage_count.
 * @param start_time_us Stage start timestamp in microseconds.
 * @param end_time_us Stage end timestamp in microseconds.
 */
static inline void esp_rt_diag_stage_record(esp_rt_diag_t *diagnostics,
                                            uint8_t stage_id,
                                            int64_t start_time_us,
                                            int64_t end_time_us) {
#if CONFIG_ESP_RT_DIAGNOSTICS_ENABLE &&                                        \
    CONFIG_ESP_RT_DIAGNOSTICS_DETAILED_TIMING
  if (diagnostics != NULL && stage_id < diagnostics->config.stage_count) {
    /* Accumulate the sample without floating-point work in the real-time path.
     */
    uint32_t duration_us = (uint32_t)(end_time_us - start_time_us);
    esp_rt_diag_stage_snapshot_t *stage = &diagnostics->stages[stage_id];
    stage->calls++;
    stage->total_duration_us += duration_us;
    esp_rt_diag_update_max(&stage->max_duration_us, duration_us);

    /* A zero budget disables only violation counting, not stage measurement. */
    uint32_t budget_us = diagnostics->config.stage_budget_us[stage_id];
    if (budget_us > 0U && duration_us > budget_us) {
      stage->over_budget_calls++;
    }
  }
#else
  (void)diagnostics;
  (void)stage_id;
  (void)start_time_us;
  (void)end_time_us;
#endif
}

/**
 * Finish a detailed stage measurement using the component clock.
 *
 * External convenience API; not called internally. Calls
 * esp_rt_diag_stage_record() with esp_timer_get_time(). Pair it with
 * esp_rt_diag_stage_begin().
 * @param diagnostics Initialized task-owned accumulator; NULL is a no-op.
 * @param stage_id Zero-based ID smaller than configured stage_count.
 * @param start_time_us Value previously returned by stage_begin().
 */
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

/**
 * Record one already-measured application interval.
 *
 * External hot-path API; not called internally. Updates only sample count,
 * minimum, and maximum. The interval is expressed in microseconds. Invalid IDs
 * are ignored and the function is a no-op unless detailed timing is enabled.
 * @param diagnostics Initialized task-owned accumulator; NULL is a no-op.
 * @param interval_id Zero-based ID smaller than configured interval_count.
 * @param interval_us Already measured interval in microseconds.
 */
static inline void esp_rt_diag_interval(esp_rt_diag_t *diagnostics,
                                        uint8_t interval_id,
                                        uint32_t interval_us) {
#if CONFIG_ESP_RT_DIAGNOSTICS_ENABLE &&                                        \
    CONFIG_ESP_RT_DIAGNOSTICS_DETAILED_TIMING
  if (diagnostics != NULL && interval_id < diagnostics->config.interval_count) {
    /* Initialize the minimum from the first sample, then maintain both bounds.
     */
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
