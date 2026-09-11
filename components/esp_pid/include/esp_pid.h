#ifndef ESP_PID_H_
#define ESP_PID_H_

/**
 * @file esp_pid.h
 *
 * All public function definitions and their private call path are placed in
 * internal instruction RAM with IRAM_ATTR. The attribute intentionally appears
 * on the definitions rather than these declarations because ESP-IDF 6 assigns
 * a unique linker section to each macro expansion.
 */

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Signal differentiated by the derivative term. */
typedef enum {
  /** Differentiate the error. Reference steps can produce derivative kick. */
  ESP_PID_DERIVATIVE_ON_ERROR = 0,
  /** Differentiate the measurement and negate it. Avoids derivative kick. */
  ESP_PID_DERIVATIVE_ON_MEASUREMENT,
} esp_pid_derivative_source_t;

/** Immutable tuning and protection settings copied into each PID instance. */
typedef struct {
  float kp;
  float ki;
  float kd;
  float output_min;
  float output_max;
  /** First-order low-pass time constant for the derivative; zero disables it.
   */
  float derivative_filter_tau_s;
  /** Back-calculation tracking time; zero disables anti-windup tracking. */
  float anti_windup_tracking_time_s;
  /** Dynamic terms are held when dt is at or above this value; zero disables.
   */
  float maximum_dt_s;
  esp_pid_derivative_source_t derivative_source;
} esp_pid_config_t;

/** Mutable state and diagnostic terms belonging to one PID instance. */
typedef struct {
  float error;
  float proportional_term;
  float integral_term;
  float derivative_term;
  float filtered_derivative;
  float previous_error;
  float previous_measurement;
  float unsaturated_output;
  float output;
  bool previous_input_valid;
  bool last_dt_valid;
} esp_pid_state_t;

/** Self-contained PID instance; no dynamic allocation is used. */
typedef struct {
  esp_pid_config_t config;
  esp_pid_state_t state;
  bool initialized;
} esp_pid_t;

/**
 * @brief Initialize a PID instance from a configuration.
 *
 * The function validates every configuration field, copies the configuration
 * into @p pid, clears all dynamic state, and marks the instance as initialized.
 * The configuration supplied by the caller may therefore be discarded after
 * this function returns.
 *
 * This is a public entry point intended to be called by application code. No
 * other public or internal function in this component calls it. Internally it
 * calls the private `valid_config()` helper.
 *
 * @param[out] pid Instance to initialize.
 * @param[in] config Tuning, saturation, filtering, and protection settings.
 * @return ESP_OK on success, or ESP_ERR_INVALID_ARG if either pointer or any
 *         configuration field is invalid.
 */
esp_err_t esp_pid_init(esp_pid_t *pid, const esp_pid_config_t *config);

/**
 * @brief Reset the dynamic state while retaining the configuration.
 *
 * This clears the integral accumulator, derivative history and filter, previous
 * samples, diagnostic terms, and latest output. The copied gains and limits are
 * unchanged, and the instance remains initialized.
 *
 * This is a public entry point intended to be called by application code. It
 * is not called by another function inside this component.
 *
 * @param[in,out] pid Previously initialized PID instance.
 * @return ESP_OK on success, or ESP_ERR_INVALID_STATE if @p pid is null or has
 *         not been initialized.
 */
esp_err_t esp_pid_reset(esp_pid_t *pid);

/**
 * @brief Calculate one saturated PID output sample.
 *
 * The function calculates the error and proportional term, updates the
 * filtered derivative, integrates the error, applies optional anti-windup by
 * back-calculation, and clamps the result to the configured actuator limits.
 * The derivative may act on the error or on the negated measurement. An
 * invalid sample interval holds the integral and derivative filter states but
 * still refreshes the proportional term and bounded output.
 *
 * This is a public entry point intended to be called once per controller
 * iteration by application code. It is not called by another function inside
 * this component. Internally it calls the private `clamp()` helper when
 * calculating the realizable and final outputs.
 *
 * @param[in,out] pid Previously initialized PID instance.
 * @param[in] reference Desired value of the controlled variable.
 * @param[in] measurement Measured value of the controlled variable.
 * @param[in] dt_s Time since the previous update, in seconds.
 * @param[out] output Saturated action calculated by the controller.
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE for an invalid instance or
 *         output pointer, or ESP_ERR_INVALID_ARG for a non-finite numeric
 *         input.
 */
esp_err_t esp_pid_update(esp_pid_t *pid, float reference, float measurement,
                         float dt_s, float *output);

/**
 * @brief Copy the latest controller state for telemetry or diagnostics.
 *
 * The returned snapshot includes the P, I, and D terms; derivative history;
 * unsaturated and saturated outputs; latest error; and timing validity flags.
 * It does not modify the PID instance.
 *
 * This is a public entry point intended to be called by application code. It
 * is not called by another function inside this component.
 *
 * @param[in] pid Previously initialized PID instance.
 * @param[out] state Destination that receives a copy of the current state.
 * @return ESP_OK on success, or ESP_ERR_INVALID_STATE if a pointer is null or
 *         the instance has not been initialized.
 */
esp_err_t esp_pid_get_state(const esp_pid_t *pid, esp_pid_state_t *state);

#ifdef __cplusplus
}
#endif

#endif /* ESP_PID_H_ */
