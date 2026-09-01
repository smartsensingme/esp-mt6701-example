# esp_rt_diagnostics

Development-time instrumentation for deterministic ESP-IDF tasks. The
component accumulates bounded timing statistics and counters in task-owned
static memory and produces immutable snapshots on demand.

It deliberately does not create timers or tasks, access sensors, transport
samples, format logs, or record time series. Applications own those policies
and may attach their own instantaneous control-state payload to each diagnostic
snapshot.

## Ownership and real-time behavior

Each `esp_rt_diag_t` has exactly one task owner. The hot-path API uses no heap,
lock, queue, string lookup, or logging. Snapshot extraction must be performed by
that same task. The snapshot may then be copied to a lower-priority task.

Wake latency is recorded only when exactly one periodic event is pending. When
events accumulate, a single latest ISR timestamp cannot identify the first
delayed event reliably; the component reports the excess through
`missed_events` instead of publishing a misleading latency value.

Disable `CONFIG_ESP_RT_DIAGNOSTICS_ENABLE` to compile the inline hot-path
instrumentation into no-ops.

## Typical integration

The application assigns compact IDs; the component never looks up names in the
critical path:

```c
enum { STAGE_I2C, STAGE_CONTROL, STAGE_COUNT };
enum { EVENT_SENSOR_OK, EVENT_SENSOR_ERROR, EVENT_COUNT };
enum { INTERVAL_SAMPLE, INTERVAL_COUNT };

esp_rt_diag_t diagnostics;
ESP_ERROR_CHECK(esp_rt_diag_init(
    &diagnostics,
    &(esp_rt_diag_config_t) {
        .expected_period_us = 250,
        .deadline_us = 250,
        .stage_count = STAGE_COUNT,
        .event_count = EVENT_COUNT,
        .interval_count = INTERVAL_COUNT,
    },
    esp_timer_get_time()));
```

The ISR captures only the event timestamp. The awakened task owns every other
update:

```c
// ISR
esp_rt_diag_isr_capture(&last_isr_time_us);

// Periodic task
esp_rt_diag_cycle_begin_from_isr(&diagnostics, &last_isr_time_us,
                                 pending_events);

int64_t start_us = esp_rt_diag_stage_begin();
esp_err_t result = read_sensor();
esp_rt_diag_stage_end(&diagnostics, STAGE_I2C, start_us);
esp_rt_diag_event(&diagnostics,
                  result == ESP_OK ? EVENT_SENSOR_OK : EVENT_SENSOR_ERROR, 1);

int64_t cycle_end_us = esp_rt_diag_cycle_end_now(&diagnostics);
if (esp_rt_diag_snapshot_due(&diagnostics, cycle_end_us)) {
    esp_rt_diag_snapshot_t snapshot;
    ESP_ERROR_CHECK(esp_rt_diag_take_snapshot(
        &diagnostics, cycle_end_us, &snapshot));
    publish_without_waiting(&snapshot);
}
```

One context supports up to eight stages, eight application events, and four
intervals. Multiple deterministic tasks use independent contexts.
