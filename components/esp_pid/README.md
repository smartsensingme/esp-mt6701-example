# `esp_pid`

`esp_pid` is a reusable, allocation-free PID controller for ESP-IDF. Each
`esp_pid_t` object owns a copy of its configuration and all dynamic state, so an
application can create several independent control loops without global state
inside the component.

The component calculates a bounded control action. It does not access PWM,
GPIO, motors, heaters, valves, or any other hardware. The application passes a
reference and a measurement to the PID and decides how to apply the returned
action to its actuator.

## Features

- configurable output saturation;
- integral action with optional back-calculation anti-windup;
- derivative on measurement or on error;
- first-order low-pass filtering of the differentiated signal;
- protection against invalid or implausible sample intervals;
- access to the P, I, and D terms and to saturated/unsaturated outputs;
- no dynamic memory allocation.

## Internal-memory and deterministic-loop use

All public PID functions and their private helpers are marked `IRAM_ATTR`.
Their executable code is therefore linked into the ESP32 internal instruction
RAM instead of being fetched from external flash through the cache. This
includes `esp_pid_update()` and `esp_pid_get_state()`, which the current control
loop calls every iteration, as well as initialization and reset operations.

Code placement alone is not a complete cache-independent guarantee. The
`esp_pid_t` instance, the output destination, and any configuration passed to
`esp_pid_init()` must also be accessible from internal RAM while their
respective functions execute. Do not place a real-time PID instance in PSRAM.
An instance allocated on the stack of a task created with the ordinary ESP-IDF
FreeRTOS task APIs uses internal RAM by default, as in this project.

Single-precision division and structure copies may compile into `__divsf3` and
`memcpy`. On the ESP32-S3/ESP-IDF configuration used by this project, both are
provided by internal ROM. The build map should be checked again when porting
the component to another target or toolchain.

`IRAM_ATTR` does not make an API interrupt-safe, reentrant, or thread-safe. A
single PID instance must still have one owner, and IRAM should be reserved for
genuinely time-critical code because it is a limited resource.

## Adding the component to an application

Place `esp_pid` under the project's `components` directory. In the component
that uses it, add `esp_pid` to `REQUIRES` or `PRIV_REQUIRES`:

```cmake
idf_component_register(
    SRCS "my_controller.c"
    INCLUDE_DIRS "."
    REQUIRES esp_pid
)
```

Then include its public header:

```c
#include "esp_pid.h"
```

## Configuration: `esp_pid_config_t`

All fields use the units chosen by the application. If the controlled variable
is speed in RPM and the output is duty cycle in percent, for example, the error
is in RPM and the output is in percent.

| Field | Meaning |
|---|---|
| `kp` | Proportional gain. The proportional term is `kp * error`. Its units are output units per input unit, such as `%/RPM`. |
| `ki` | Integral gain. The integrator accumulates `ki * error * dt_s`. Its units are output units per input-unit-second, such as `%/(RPM.s)`. Set it to zero for a PD or P controller. |
| `kd` | Derivative gain. Its units are output-unit-seconds per input unit, such as `%.s/RPM`. Set it to zero for a PI or P controller. |
| `output_min` | Smallest command the actuator is allowed to receive. It must be less than `output_max`. Use `-100` for full reverse, for example, or `0` for a unidirectional actuator. |
| `output_max` | Largest command the actuator is allowed to receive. It must be greater than `output_min`. |
| `derivative_filter_tau_s` | Time constant, in seconds, of the first-order low-pass filter applied to the differentiated signal. A larger value filters more noise but adds more delay. Zero disables filtering. It cannot be negative. |
| `anti_windup_tracking_time_s` | Tracking time, in seconds, of the back-calculation anti-windup. A smaller positive value removes accumulated integral action more aggressively during saturation. Zero disables back-calculation. It cannot be negative. Choose a value safely larger than the sample interval. |
| `maximum_dt_s` | Largest acceptable sample interval, in seconds. Integration and derivative-filter updates are held when `dt_s <= 0` or `dt_s >= maximum_dt_s`. Zero disables only the upper limit; `dt_s` must still be positive. |
| `derivative_source` | Selects whether the derivative is calculated from the error or from the measurement. The available values are explained below. |

Every floating-point field must be finite: `NaN` and infinity are rejected by
`esp_pid_init()`.

### Choosing the derivative source

`ESP_PID_DERIVATIVE_ON_MEASUREMENT` calculates the derivative contribution as
the negative derivative of the measurement. This is usually preferable for a
setpoint controller because an instantaneous reference step does not cause a
large derivative kick.

`ESP_PID_DERIVATIVE_ON_ERROR` differentiates `reference - measurement`. This
also reacts to changes in the reference and may be useful when that behavior is
intentional.

## Complete usage example

The instance normally has a lifetime at least as long as the control loop. Do
not create and initialize it again on every iteration, because that would erase
the integrator and derivative history.

```c
#include "esp_pid.h"

static esp_pid_t speed_pid;

static const esp_pid_config_t speed_pid_config = {
    .kp = 0.25f,
    .ki = 3.0f,
    .kd = 0.0001f,
    .output_min = -100.0f,
    .output_max = 100.0f,
    .derivative_filter_tau_s = 0.020f,
    .anti_windup_tracking_time_s = 0.20f,
    .maximum_dt_s = 0.010f,
    .derivative_source = ESP_PID_DERIVATIVE_ON_MEASUREMENT,
};

esp_err_t speed_controller_init(void)
{
    return esp_pid_init(&speed_pid, &speed_pid_config);
}

esp_err_t speed_controller_step(float reference_rpm,
                                float measured_rpm,
                                float dt_s)
{
    float command_percent = 0.0f;
    esp_err_t err = esp_pid_update(&speed_pid, reference_rpm, measured_rpm,
                                   dt_s, &command_percent);
    if (err != ESP_OK) {
        return err;
    }

    /* The actuator driver, not esp_pid, interprets the command. */
    if (command_percent == 0.0f) {
        motor_brake();
    } else {
        motor_set_speed(command_percent);
    }
    return ESP_OK;
}
```

For a 1 kHz loop, the nominal `dt_s` is `0.001f`. It is better to pass the
measured elapsed time when scheduling jitter matters. The `motor_brake()` and
`motor_set_speed()` names above are placeholders for the application's driver.

## API

### `esp_pid_init()`

```c
esp_err_t esp_pid_init(esp_pid_t *pid, const esp_pid_config_t *config);
```

Validates the configuration, copies it into the instance, clears all state, and
marks the instance as initialized. Because the configuration is copied, the
original structure does not need to remain alive.

Returns:

- `ESP_OK` on success;
- `ESP_ERR_INVALID_ARG` if a pointer is null or the configuration is invalid.

### `esp_pid_update()`

```c
esp_err_t esp_pid_update(esp_pid_t *pid,
                         float reference,
                         float measurement,
                         float dt_s,
                         float *output);
```

Executes one controller iteration. The function calculates

```text
error = reference - measurement
output = saturate(P + I + D, output_min, output_max)
```

and stores the saturated action through the `output` pointer. The function's
return value is an ESP-IDF error code, not the control action.

Returns:

- `ESP_OK` when an output was calculated;
- `ESP_ERR_INVALID_STATE` if the instance/output pointer is invalid or the
  instance was not initialized;
- `ESP_ERR_INVALID_ARG` if `reference`, `measurement`, or `dt_s` is not finite.

A finite but unacceptable `dt_s` is not an API error. The function returns
`ESP_OK`, updates the proportional term and output, holds the integral and
derivative-filter states, and reports `last_dt_valid = false` in its state.

### `esp_pid_reset()`

```c
esp_err_t esp_pid_reset(esp_pid_t *pid);
```

Clears the integrator, derivative filter and history, previous inputs,
diagnostic terms, and latest output. Gains, limits, and the initialized state
are preserved. Call it when starting an independent experiment, after a major
operating-mode change, or whenever old controller memory must not affect a new
run.

Returns `ESP_OK` or `ESP_ERR_INVALID_STATE` for a null/uninitialized instance.

### `esp_pid_get_state()`

```c
esp_err_t esp_pid_get_state(const esp_pid_t *pid, esp_pid_state_t *state);
```

Copies a diagnostic snapshot without changing the controller. The most useful
fields are:

| Field | Meaning |
|---|---|
| `error` | Latest `reference - measurement`. |
| `proportional_term` | Latest proportional contribution. |
| `integral_term` | Current integrator value after anti-windup correction. |
| `derivative_term` | Latest filtered derivative contribution, including its configured sign. |
| `filtered_derivative` | Internal filtered derivative before multiplication by `kd` and before the measurement-derivative sign is applied. |
| `previous_error` | Error retained for the next derivative calculation. |
| `previous_measurement` | Measurement retained for the next derivative calculation. |
| `unsaturated_output` | Sum `P + I + D` before the final clamp. |
| `output` | Command after clamping to the configured limits. |
| `previous_input_valid` | Indicates that derivative history has been initialized. |
| `last_dt_valid` | Indicates whether the last interval was accepted for dynamic updates. |

Returns `ESP_OK` or `ESP_ERR_INVALID_STATE` for an invalid pointer or
uninitialized instance.

## Saturation and anti-windup

The output limits model the command range the actuator can actually realize.
When the requested `P + I + D` exceeds that range, the output is clamped. With
back-calculation enabled, the difference between the clamped and requested
outputs also drives the integrator back toward a compatible value:

```text
I <- I + Ki * error * dt
I <- I + (realizable_output - requested_output) * dt / tracking_time
```

This reduces integral windup but does not limit motor current, temperature,
voltage, acceleration, or mechanical force. Those protections must be provided
by the application and actuator driver.

## Practical notes

- Use one `esp_pid_t` instance per control loop.
- Call `esp_pid_init()` once before the first update.
- Call `esp_pid_update()` at a reasonably regular rate.
- Keep all signals and gains dimensionally consistent.
- Start tuning with `ki = 0` and `kd = 0`, tune `kp`, and then introduce the
  integral and derivative actions deliberately.
- Do not update the same instance concurrently from multiple tasks without
  synchronization.
- Direction-change dead time, brake/coast choice, PWM generation, and hardware
  fault handling belong to the actuator layer.

Portuguese documentation: [README.pt-br.md](README.pt-br.md).
