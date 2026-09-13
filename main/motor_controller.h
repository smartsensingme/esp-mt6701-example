#ifndef MOTOR_CONTROLLER_H_
#define MOTOR_CONTROLLER_H_

#include "esp_pid.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum {
  /** Safe inactive state: update() always requests zero percent. */
  MOTOR_CONTROLLER_MODE_IDLE = 0,
  /** Alternating 600/900 RPM profile controlled by esp_pid. */
  MOTOR_CONTROLLER_MODE_CLOSED_LOOP,
  /** Fixed low/high/low duty sequence used to calibrate the angle sensor. */
  MOTOR_CONTROLLER_MODE_OPEN_LOOP_TEST,
} motor_controller_mode_t;

/**
 * @brief Volatile, user-selectable parameters for one closed-loop experiment.
 *
 * All values must be finite. motor_controller_config_is_valid() enforces the
 * protocol bounds. The structure owns no dynamic resources and may be copied
 * by value when an ARM request crosses from the USB task to the control task.
 */
typedef struct {
  /** Proportional gain in %/RPM; accepted range 0..100. */
  float kp;
  /** Integral gain in %/(RPM.s); accepted range 0..1000. */
  float ki;
  /** Derivative gain in %.s/RPM; accepted range 0..10. */
  float kd;
  /** Time between 600/900 RPM steps in seconds; range 0.1..3600. */
  float reference_step_period_s;
} motor_controller_config_t;

/**
 * @brief Complete mutable state of one application motor-controller instance.
 *
 * The Core 1 real-time task owns this structure. Callers initialize it before
 * use and must not modify its fields concurrently. Fields are public for static
 * allocation and diagnostics, but normal operation uses the functions below.
 */
typedef struct {
  /** Reusable PID instance, including integrator and derivative-filter state.
   */
  esp_pid_t pid;
  /** Configuration copied when the current profile was started. */
  motor_controller_config_t config;
  /** Current closed-loop reference in RPM; zero in open-loop mode. */
  float reference_rpm;
  /** Seconds accumulated within the current reference or duty stage. */
  float profile_elapsed_s;
  /** Last signed command in percent, limited by the PID to -100..100. */
  float output_percent;
  /** Number of completed automatic closed-loop reference transitions. */
  uint32_t reference_step_count;
  /** Zero-based open-loop duty stage; 3 denotes the terminal zero stage. */
  uint8_t open_loop_stage;
  /** Currently selected controller/profile mode. */
  motor_controller_mode_t mode;
  /** Selects 900 RPM when true and 600 RPM when false. */
  bool high_reference_active;
  /** Distinguishes an armed/running calibration from its pre-start state. */
  bool open_loop_test_started;
} motor_controller_t;

/**
 * @brief Read-only status copied from a motor_controller_t for logging/capture.
 *
 * motor_controller_get_status() fills every field. The snapshot owns its values
 * and may safely be passed to another task after the call.
 */
typedef struct {
  /** Active speed reference in RPM. */
  float reference_rpm;
  /** PID reference minus measurement error in RPM. */
  float error_rpm;
  /** Latest proportional contribution in percent. */
  float proportional_term;
  /** Latest integral contribution in percent. */
  float integral_term;
  /** Latest filtered derivative contribution in percent. */
  float derivative_term;
  /** Latest saturated signed motor command in percent. */
  float output_percent;
  /** Count of completed closed-loop reference changes. */
  uint32_t reference_step_count;
  /** Zero-based open-loop stage, meaningful when open_loop_test is true. */
  uint8_t open_loop_stage;
  /** True when mode is MOTOR_CONTROLLER_MODE_OPEN_LOOP_TEST. */
  bool open_loop_test;
  /** True after the open-loop sequence has explicitly been started. */
  bool open_loop_test_started;
} motor_controller_status_t;

/**
 * @brief Copy the firmware defaults used after every boot.
 *
 * Called externally by realtime_loop_start(), CONTROL DEFAULTS, and callers
 * needing a baseline. A null destination is ignored.
 *
 * @param config Destination owned by the caller.
 */
void motor_controller_get_default_config(motor_controller_config_t *config);

/**
 * @brief Validate finite gains and the application protocol ranges.
 *
 * Called externally by CONTROL SET and internally by the profile-start
 * functions. The broad accepted ranges prevent malformed data; they do not
 * guarantee that a particular motor will be stable or safe.
 *
 * @param config Configuration to inspect; not modified or retained.
 * @return true for a non-null configuration within every documented range.
 */
bool motor_controller_config_is_valid(const motor_controller_config_t *config);

/**
 * @brief Initialize a controller in IDLE using the firmware defaults.
 *
 * Called by realtime_task() before starting the scheduler. A null pointer is
 * ignored. Reinitializing discards prior PID and profile state.
 *
 * @param controller Caller-owned instance with control-task lifetime.
 */
void motor_controller_init(motor_controller_t *controller);

/**
 * @brief Start a fresh alternating 600/900 RPM closed-loop experiment.
 *
 * Called by realtime_task() when it consumes a closed-loop ARM request. It
 * copies the volatile configuration and resets all PID, integrator, derivative,
 * reference-phase, and output state.
 *
 * @param controller Initialized instance to reset and start.
 * @param configuration Valid RAM configuration copied by value.
 * @return true when both pointers and all parameters are valid; false without
 * changing the controller otherwise.
 */
bool motor_controller_start_closed_loop_test(
    motor_controller_t *controller,
    const motor_controller_config_t *configuration);

/**
 * @brief Start or restart the Kconfig-defined open-loop calibration profile.
 *
 * Called by realtime_task() for a CAL START request. It preserves a valid
 * volatile configuration for later use but resets PID and profile state. A
 * null controller is ignored.
 *
 * @param controller Initialized control-task-owned instance.
 */
void motor_controller_start_open_loop_test(motor_controller_t *controller);

/**
 * @brief Stop the active profile and place the controller in IDLE.
 *
 * Called by realtime_task() after it consumes a host CONTROL STOP request. The
 * current validated configuration is retained for diagnostics, while PID,
 * profile, reference, and output state are reset. A null pointer is ignored.
 *
 * @param controller Initialized control-task-owned instance.
 */
void motor_controller_stop(motor_controller_t *controller);

/**
 * @brief Calculate the 1 kHz motor command.
 *
 * Output remains in COAST until a test is explicitly started. Closed-loop mode
 * alternates the reference between 600 and 900 RPM. Calibration mode follows
 * the Kconfig-selected low/high/low duty profile and returns to COAST.
 *
 * Called only by realtime_task() at 1 kHz. It performs no allocation or I/O.
 * The caller must provide exclusive access to the instance.
 *
 * @param controller Mutable controller state, or null for a zero command.
 * @param measured_speed_rpm Kalman speed estimate in RPM.
 * @param dt Actual elapsed time since the previous control update, in seconds.
 * @return Signed motor command in percent, limited to -100..100.
 */
float motor_controller_update(motor_controller_t *controller,
                              float measured_speed_rpm, float dt);

/**
 * @brief Copy the current reference, PID terms, profile state, and output.
 *
 * Called by realtime_task() for capture and by
 * publish_telemetry_snapshot() for reporting. Either null argument makes the
 * function a no-op. The function calls esp_pid_get_state().
 *
 * @param controller Controller to inspect; not modified or retained.
 * @param status Caller-owned destination filled on success.
 */
void motor_controller_get_status(const motor_controller_t *controller,
                                 motor_controller_status_t *status);

#endif /* MOTOR_CONTROLLER_H_ */
