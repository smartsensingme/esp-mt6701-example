# Guidance for AI agents instrumenting deterministic tasks

Use this file when modifying or reviewing `esp_rt_diagnostics`. Its scope
includes this component and its files. Read the public header, implementation,
Kconfig, and README before changing behavior.

The companion `../esp_rt_diagnostics_reporter` component moves completed
snapshots to a non-real-time presentation task. Read its `AGENTS.md` when the
task involves log formatting, quiet/log-affected classification, application
payloads, asynchronous publication, or interpretation of collected evidence.

## Objective

Instrument a deterministic task with bounded overhead and measurements whose
meaning is unambiguous. The accumulator must remain independent of sensors,
buses, controllers, logging, and transport policy.

Correct instrumentation comes before interpretation. A precise report cannot
repair a misplaced timestamp, an unmatched cycle boundary, or a diagnostic
cycle configured for the wrong periodic event.

## Preserve the deterministic boundary

- Keep the hot path free of logging, string formatting, dynamic allocation,
  queues, mutexes, and application callbacks.
- Keep application-specific names and presentation outside this component.
- Do not add sensor-, SPI-, I2C-, ADC-, motor-, controller-, or estimator-specific
  fields to generic diagnostic structures.
- Use numeric IDs for stages, events, and intervals. Translate IDs to names only
  after copying a completed snapshot out of the monitored task.
- Preserve the single-owner model: one real-time task owns and updates one
  `esp_rt_diag_t` instance. Create independent instances for independent tasks.
- Except for `esp_rt_diag_isr_capture()`, do not update an instance from an ISR
  or another task.
- Snapshot memory is value-owned after extraction and may be copied elsewhere.
  Never let another task read the live accumulator concurrently.

## Define the measured cycle first

Before adding calls, identify the periodic event represented by one diagnostic
cycle:

1. Name the hardware/software event that releases the task.
2. State its exact nominal frequency and period.
3. Identify the latest acceptable completion time: the deadline.
4. List all work performed once per release.
5. Identify slower work performed by a divider inside the faster task.

`expected_period_us` describes the event that calls `cycle_begin`, not every
sub-operation in the task. For example, if sensor acquisition executes at 3 kHz
and control executes every third acquisition, configure the diagnostic cycle
for the approximately 333.333 us acquisition period. Record the 1 kHz control
spacing as an interval and the controller itself as a conditional stage.

When the exact period is fractional, document the rounding policy. Use the
nearest integer for descriptive expected-rate metadata when appropriate, and a
deadline that does not accidentally become stricter solely because of integer
rounding.

## Configure the instance correctly

### Period and deadline

- `expected_period_us` must be nonzero and represents nominal release spacing.
- `deadline_us` must be nonzero and represents allowed wake latency plus task
  processing. It may be smaller than, equal to, or larger than the period when
  the scheduling design explicitly requires that.
- Do not silently enlarge the deadline until overruns disappear. Derive it from
  the system requirement.

### Measurement window

- Select `window_duration_us` long enough to contain many cycles and short
  enough to correlate disturbances with application state.
- Zero selects `CONFIG_ESP_RT_DIAGNOSTICS_WINDOW_MS`.
- Five seconds is a reasonable initial development window for kHz loops, but it
  is not a universal requirement.
- Ensure counters cannot overflow within the chosen window. Counters use
  unsigned arithmetic without saturation.

### Capacity and IDs

- `stage_count`, `event_count`, and `interval_count` define the valid ID ranges,
  not merely array allocation sizes.
- Keep enumeration values contiguous from zero and terminate them with a count
  enumerator.
- Maximum capacities are eight stages, eight events, and four intervals.
- Invalid IDs are ignored by hot-path helpers. Therefore, verify enum/count
  consistency during review rather than expecting a runtime error.

### Stage budgets

Allocate the cycle deadline across meaningful stages. A budget should express a
useful engineering limit, not just the largest value seen in one experiment.

`stage_budget_us[id] == 0` disables only violation counting. The stage is still
measured when detailed timing is enabled. A stage violation does not
automatically imply a whole-cycle deadline overrun, and the converse is also
possible when several individually acceptable stages add up to too much time.

## Correct placement of cycle instrumentation

The normal sequence is:

```text
timer ISR
  capture event timestamp
  notify task

monitored task
  consume notification count
  cycle_begin_from_isr
  perform and instrument work
  cycle_end_now
  test and extract snapshot
  publish copied snapshot outside the measured cycle
```

### ISR side

Call `esp_rt_diag_isr_capture()` close to the task notification. The stored low
32-bit timestamp represents the most recent release. Keep the destination in
persistent volatile storage shared with the owner task.

Do not perform stage timing, logging, snapshot extraction, or reporter
publication in the ISR. The unsigned low-32-bit latency subtraction is valid
across timer wrap provided the actual ISR-to-task delay is less than one wrap
(approximately 71.6 minutes).

### Task start

Immediately after `ulTaskNotifyTake()` or the equivalent release operation,
call `esp_rt_diag_cycle_begin_from_isr()` with:

- the accumulator owned by the task;
- the volatile ISR timestamp address;
- the actual number of notifications consumed.

Exactly one pending event permits valid wake-latency measurement. More than one
means events accumulated; the component records `pending_events - 1` losses and
does not claim that the latest ISR timestamp represents every missed release.
Zero opens processing measurement without valid wake timing.

### Task end

Call `esp_rt_diag_cycle_end()` or `_now()` exactly once on every path after a
successful begin. This includes sensor errors, invalid measurements, skipped
control-divider iterations, and recoverable failures.

Do not begin a new cycle while the prior one remains open. Snapshot extraction
rejects an open cycle specifically to prevent one execution from being divided
between windows.

Decide deliberately whether publication-copy cost belongs inside or outside
the measured cycle. The usual policy is to close the cycle first, extract the
snapshot, and publish it afterward so presentation handoff does not inflate
normal processing time. If publication cost is under study, measure it as a
separate stage and document that choice.

## Choosing stages, events, and intervals

### Stages: duration of work

Use a stage for a variable-duration operation executed by the owner task:

- I2C or SPI transaction;
- ADC frame processing;
- estimator/Kalman update;
- controller update;
- state-machine transition;
- snapshot extraction or publication copy.

Pair `esp_rt_diag_stage_begin()` with `esp_rt_diag_stage_end()` when the
component should read both endpoints. If the application already has accurate
timestamps, call `esp_rt_diag_stage_record()` to avoid another timer read.

Avoid overlapping stages unless overlap is intentional and documented. The sum
of overlapping stage times cannot be compared directly with processing time.
Instrument enough of the task that a large unexplained gap between stage maxima
and `max_processing_time_us` can be investigated.

### Events: occurrences and faults

Use `esp_rt_diag_event()` for counts, not durations. Examples include:

- successful sensor/estimator update;
- I2C/SPI error or timeout;
- CRC/parity failure;
- driver retry;
- controller execution;
- ADC invalid result or buffer overflow.

Record a useful denominator where possible. For example, an SPI error count is
more meaningful beside the total successful/attempted transfer count.

Do not increment a diagnostic event by logging the event inside the monitored
task. Accumulate it and let a snapshot consumer format it later.

### Intervals: spacing supplied by the application

Use `esp_rt_diag_interval()` for an interval already calculated by the
application. Document exactly what the endpoints mean:

- timer-release to timer-release;
- sensor-transaction completion to completion;
- estimator-update to estimator-update;
- controller-update to controller-update.

A sample-completion interval includes transaction-duration variation; a timer
release interval does not describe the same phenomenon. Do not compare them as
if they were interchangeable.

## Snapshot lifecycle

1. Initialize once with `esp_rt_diag_init()` before the periodic source starts.
2. After closing each cycle, use `esp_rt_diag_snapshot_due()` as the inline
   frequent-path test.
3. When due, call `esp_rt_diag_try_take_snapshot()` and check both its return
   value and `snapshot_taken`.
4. Copy or publish the completed `esp_rt_diag_snapshot_t`; never share the live
   accumulator.
5. Continue with the automatically opened next window.

`esp_rt_diag_take_snapshot()` is the unconditional alternative. Both snapshot
operations preserve lifetime totals while resetting window data and incrementing
`window_id`. Reinitialization erases lifetime history.

Do not log snapshot errors synchronously from a hard real-time loop. Count or
store the error in a bounded way and report it outside that path.

## Kconfig and instrumentation cost

- `CONFIG_ESP_RT_DIAGNOSTICS_ENABLE` enables cycle, wake, missed-event,
  deadline, and event accounting.
- `CONFIG_ESP_RT_DIAGNOSTICS_DETAILED_TIMING` additionally enables stages and
  intervals. Stage-begin/end pairs read `esp_timer` and therefore add measurable
  overhead.
- Disabling diagnostics compiles the inline hot-path helpers into no-ops. Keep
  application objects and calls compile-safe in both configurations.

The inline declarations reduce call overhead and permit compile-time removal;
they do not justify adding unbounded work. Do not assume every caller is safe
during flash/cache-disabled intervals merely because a helper is inline. ISR
and IRAM safety depend on the complete inlined call graph, data placement, ESP-IDF
timer guarantees, build configuration, and the ISR that contains the call.

When timing margins are small, compare:

1. detailed timing enabled;
2. base diagnostics enabled but detailed timing disabled;
3. diagnostics completely disabled.

This separates application timing from the cost of observation.

## Review checklist

Before accepting an instrumentation change, verify:

- the measured cycle matches the configured period;
- period/deadline rounding is documented;
- ISR capture is adjacent to the release notification;
- cycle begin occurs before measured task work;
- every begin has exactly one end on all control-flow paths;
- snapshots are taken only after cycle end;
- enum counts match configuration counts;
- each stage budget has units and rationale;
- interval endpoints are explicitly defined;
- application errors have meaningful event counters;
- no logs, heap allocation, blocking transport, or formatting entered the hot
  path;
- the accumulator has exactly one task owner;
- builds succeed with relevant diagnostic Kconfig combinations;
- `git diff --check` is clean.

## Common instrumentation mistakes

| Mistake | Consequence | Correction |
|---|---|---|
| Configuring 1 kHz while beginning a cycle at 3 kHz | Expected rate and deadline interpretation are wrong. | Configure the acquisition cycle and record 1 kHz control as an interval/stage. |
| Using the notification count as a Boolean | Accumulated releases and true losses disappear. | Pass the full count to `cycle_begin_from_isr()`. |
| Ending only successful cycles | Processing statistics become biased and snapshots may fail as open. | Close every begun cycle on error paths too. |
| Logging a sensor failure immediately | The diagnostic mechanism perturbs the task it measures. | Increment an event and report the snapshot asynchronously. |
| Measuring only total processing | A violation is visible but cannot be localized. | Add bounded stages around variable-duration operations. |
| Measuring every tiny operation | Timer-read overhead dominates and obscures behavior. | Instrument architectural stages with actionable budgets. |
| Treating zero stage budget as zero allowed time | Misinterprets the API. | Zero disables violation counting for that stage. |
| Sharing one accumulator across tasks | Data races corrupt counts and state. | Use one instance per owner task. |
| Taking a snapshot before cycle end | The component rejects it or the boundary becomes ambiguous. | End first, snapshot second. |
| Changing control/filter gains to cure missed releases | Changes plant behavior without addressing timing. | Diagnose scheduler, bus, driver, and workload timing independently. |

## Handoff to the reporter and AI interpretation

This component establishes facts; it does not assign causes. After a correct
snapshot is produced, use `../esp_rt_diagnostics_reporter/AGENTS.md` to:

- keep presentation off the monitored task;
- compare `quiet` and `log-affected` windows;
- interpret missed events, overruns, wake latency, processing, and stage data;
- test interference from console logs, USB polling, capture, and other tasks;
- report conclusions with denominators, repeated windows, and falsifiable next
  experiments.

When an observed result is surprising, inspect instrumentation placement before
tuning the application. A wrong measurement boundary can imitate scheduler
jitter, a slow bus, or a deadline failure.

## Modification discipline

- Diagnose the existing integration before changing component semantics.
- Preserve source compatibility unless the user explicitly authorizes an API
  change.
- Do not increase capacities, counter widths, or hot-path work without measuring
  memory and execution-time effects.
- Keep comments and README contracts synchronized with actual behavior.
- Format touched C sources, run the relevant enabled build, exercise disabled or
  reduced-instrumentation builds when applicable, and run `git diff --check`.
