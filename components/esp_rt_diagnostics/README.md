# esp_rt_diagnostics

`esp_rt_diagnostics` is bounded development instrumentation for periodic,
deterministic ESP-IDF tasks. It answers questions such as:

- Did the task receive every periodic event?
- How late did it wake after the timer ISR?
- How long did its processing and individual stages take?
- How often did a cycle or stage exceed its budget?
- What were the minimum and maximum application-defined intervals?

The component only accumulates numeric measurements and creates immutable
snapshots. It does not create the application timer, suspend the monitored task,
format logs, record time series, or interpret motor/sensor state. Use
`esp_rt_diagnostics_reporter` to move snapshots to a low-priority presentation
task.

AI coding agents can use `AGENTS.md` in this directory for the correct
instrumentation workflow and the reporter's `AGENTS.md` for evidence-based
interpretation and controlled timing experiments.

## Architecture and ownership

One `esp_rt_diag_t` belongs to exactly one monitored task:

```text
timer ISR                         monitored task
   |                                   |
   +-- isr_capture(timestamp)           |
                                       +-- cycle_begin_from_isr(...)
                                       +-- stage/event/interval updates
                                       +-- cycle_end_now()
                                       +-- take snapshot when due
                                                  |
                                                  +-- immutable copy to consumer
```

The accumulator has fixed-size arrays, no dynamic allocation, and no internal
locks. Except for `esp_rt_diag_isr_capture()`, update and snapshot functions
must be called by the owner task. Multiple tasks are supported by giving each
task its own instance.

The hot-path functions are `static inline`. When
`CONFIG_ESP_RT_DIAGNOSTICS_ENABLE` is disabled, their bodies compile to no-ops.
Stage and interval timing additionally depend on
`CONFIG_ESP_RT_DIAGNOSTICS_DETAILED_TIMING`.

## Concepts

### Cycle, wake latency, processing time, and deadline

A cycle starts when the periodic task wakes and ends after its work is complete:

```text
periodic ISR event             task starts                    task ends
       |---------------------------|-------------------------------|
              wake latency                 processing time
       |-----------------------------------------------------------|
                    deadline consumption / cycle time
```

The deadline is exceeded when
`wake_latency_us + processing_us > deadline_us`.

Wake latency is valid only when exactly one event is pending. If the task wakes
with `pending_events > 1`, one stored ISR timestamp cannot identify the latency
of every accumulated event. The component counts `pending_events - 1` as missed
and omits wake latency from that cycle instead of reporting a misleading number.

### Windows and lifetime counters

Measurements are grouped into windows. `window_duration_us` selects the desired
duration; zero uses `CONFIG_ESP_RT_DIAGNOSTICS_WINDOW_MS`. A snapshot reports
the actual elapsed duration and receives a monotonically increasing `window_id`
starting at 1.

Snapshot extraction resets window counters, maxima, stages, and intervals. It
does not reset lifetime totals or `lifetime_max_processing_time_us`.
Reinitializing the instance resets both window and lifetime state.

### Stages, events, and intervals

- A **stage** is a named section such as I2C, estimator, or control. Detailed
  timing records calls, total/average duration, maximum duration, and optional
  budget violations.
- An **event** is an application-defined counter such as successful estimator
  updates or sensor errors. Both window and lifetime totals are retained.
- An **interval** is an already measured duration such as time between sensor
  samples. Detailed timing retains sample count, minimum, and maximum.

The application assigns small integer IDs. No string lookup occurs in the
real-time path. Capacity is fixed at eight stages, eight events, and four
intervals per instance.

## Configuration

### Kconfig

| Option | Meaning |
|---|---|
| `CONFIG_ESP_RT_DIAGNOSTICS_ENABLE` | Enables base cycle, wake, deadline, and event instrumentation. |
| `CONFIG_ESP_RT_DIAGNOSTICS_DETAILED_TIMING` | Enables stage and interval timestamps/statistics. |
| `CONFIG_ESP_RT_DIAGNOSTICS_WINDOW_MS` | Default window duration used when an instance selects zero. |

### `esp_rt_diag_config_t`

| Field | Unit / range | Meaning |
|---|---|---|
| `expected_period_us` | microseconds, nonzero | Nominal period used as metadata and to derive the expected rate. |
| `deadline_us` | microseconds, nonzero | Maximum accepted wake plus processing time; it may differ from the period. |
| `window_duration_us` | microseconds | Per-instance window; zero selects the Kconfig default. |
| `stage_count` | 0 to 8 | Number of valid stage IDs. |
| `event_count` | 0 to 8 | Number of valid event IDs. |
| `interval_count` | 0 to 4 | Number of valid interval IDs. |
| `stage_budget_us[id]` | microseconds | Optional strict limit; zero disables only violation counting. |

Counters use unsigned arithmetic without saturation. Choose window durations and
event increments that cannot overflow `uint32_t`. Timestamps passed to one
instance must come from the same monotonic microsecond clock.

## Complete integration example

Assign stable IDs and initialize the instance before starting the periodic
timer:

```c
enum { STAGE_I2C, STAGE_ESTIMATOR, STAGE_CONTROL, STAGE_COUNT };
enum { EVENT_ESTIMATOR_UPDATE, EVENT_SENSOR_ERROR, EVENT_COUNT };
enum { INTERVAL_SAMPLE, INTERVAL_COUNT };

static esp_rt_diag_t diagnostics;
static volatile uint32_t last_timer_isr_us;

ESP_ERROR_CHECK(esp_rt_diag_init(
    &diagnostics,
    &(esp_rt_diag_config_t) {
        .expected_period_us = 1000,
        .deadline_us = 1000,
        .window_duration_us = 5000000,
        .stage_count = STAGE_COUNT,
        .event_count = EVENT_COUNT,
        .interval_count = INTERVAL_COUNT,
        .stage_budget_us = {
            [STAGE_I2C] = 300,
            [STAGE_ESTIMATOR] = 100,
            [STAGE_CONTROL] = 150,
        },
    },
    esp_timer_get_time()));
```

The timer ISR only stores the most recent event timestamp:

```c
static bool IRAM_ATTR timer_alarm_callback(/* driver arguments */)
{
    esp_rt_diag_isr_capture(&last_timer_isr_us);
    /* Notify the periodic task. */
    return higher_priority_task_woken;
}
```

The owner task updates every other measurement:

```c
uint32_t pending_events = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
esp_rt_diag_cycle_begin_from_isr(
    &diagnostics, &last_timer_isr_us, pending_events);

int64_t i2c_start_us = esp_rt_diag_stage_begin();
esp_err_t sensor_error = read_sensor();
esp_rt_diag_stage_end(&diagnostics, STAGE_I2C, i2c_start_us);

esp_rt_diag_event(
    &diagnostics,
    sensor_error == ESP_OK ? EVENT_ESTIMATOR_UPDATE : EVENT_SENSOR_ERROR,
    1U);
esp_rt_diag_interval(&diagnostics, INTERVAL_SAMPLE, measured_sample_dt_us);

int64_t cycle_end_us = esp_rt_diag_cycle_end_now(&diagnostics);
if (esp_rt_diag_snapshot_due(&diagnostics, cycle_end_us)) {
    esp_rt_diag_snapshot_t snapshot;
    bool snapshot_taken = false;
    ESP_ERROR_CHECK(esp_rt_diag_try_take_snapshot(
        &diagnostics, cycle_end_us, &snapshot, &snapshot_taken));
    if (snapshot_taken) {
        publish_copy_without_waiting(&snapshot);
    }
}
```

`esp_rt_diag_stage_begin()` and `esp_rt_diag_stage_end()` perform timer reads.
If the application already has both timestamps, use
`esp_rt_diag_stage_record()` to avoid an extra read.

## API guide

### Lifecycle and snapshots

- `esp_rt_diag_init()` copies configuration and opens window 1.
- `esp_rt_diag_snapshot_due()` is the inline, non-mutating due test intended for
  the frequent path.
- `esp_rt_diag_try_take_snapshot()` distinguishes "not due" from errors through
  `snapshot_taken`.
- `esp_rt_diag_take_snapshot()` closes a window unconditionally. It fails while
  a cycle is open, preventing a snapshot from splitting one cycle.

### Hot-path recording

- `esp_rt_diag_isr_capture()` is the only ISR-side function.
- `esp_rt_diag_cycle_begin_from_isr()` is the normal task-side companion.
- `esp_rt_diag_cycle_begin()` and `_now()` support alternate timestamp flows.
- `esp_rt_diag_cycle_end()` and `_now()` finish deadline accounting.
- `esp_rt_diag_event()` increments window and lifetime event counters.
- `esp_rt_diag_stage_begin()`, `_record()`, and `_end()` measure named stages.
- `esp_rt_diag_interval()` records an interval already calculated by the app.

### Derived snapshot values

`esp_rt_diag_cycle_rate_hz()`, `esp_rt_diag_event_rate_hz()`,
`esp_rt_diag_deadline_overrun_percent()`, and
`esp_rt_diag_stage_average_us()` are intended for non-real-time presentation.
They return zero for invalid or empty input instead of producing NaN/Inf.

## Snapshot interpretation

- `cycles` is work actually completed in the window.
- `missed_events` is the number of excess pending periodic notifications.
- `deadline_overruns` counts completed cycles whose processing time, plus valid
  wake latency, exceeded `deadline_us`.
- `max_processing_time_us` excludes scheduler wake latency.
- `max_cycle_time_us` includes wake latency only for unambiguous cycles.
- `cycles_with_valid_wake_time` shows how much evidence supports wake metrics.
- fields prefixed by `total_` and `lifetime_` cover the entire instance lifetime.

Do not compare only `max_processing_time_us` with the deadline when scheduler
latency matters; use `max_cycle_time_us` and the overrun counter together.

## Errors and troubleshooting

- `ESP_ERR_INVALID_ARG` from initialization usually means a zero period/deadline
  or too many configured IDs.
- Snapshot operations reject backward time and an open cycle. Always call a
  matching cycle-end operation, including error paths.
- No stage/interval data usually means detailed timing is disabled or the ID is
  outside the configured count.
- A zero wake-valid count with completed cycles means the task usually received
  zero or multiple pending events, so ISR latency was intentionally omitted.
- Instrumentation still has a small timer-read and arithmetic cost. Compare an
  enabled build against a disabled build when validating hard deadlines.

## Component boundary

This component should remain independent of sensors, controllers, motors,
logging formats, and transports. Application-specific state belongs in a
separate payload copied alongside the snapshot. The companion reporter provides
that mechanism without adding those dependencies to the deterministic core.

## Integration in this project

`main/realtime_loop.c` owns one accumulator in the sensor/control task. The
3 kHz sensor cycle records I2C and Kalman stages; every third cycle records the
1 kHz controller stage. The application also records sensor/control intervals
and estimator, controller, and sensor-error events. At the window boundary it
copies the completed snapshot through `main/realtime_telemetry.c`; it never
prints the diagnostic report from the monitored core.
