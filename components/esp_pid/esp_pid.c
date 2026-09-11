#include "esp_pid.h"

#include "esp_attr.h"
#include <math.h>

/**
 * @brief Restrict a scalar to a closed interval.
 *
 * This private helper is called only by `esp_pid_update()`: once to determine
 * the actuator output available to the anti-windup calculation and once to
 * produce the final bounded controller output.
 */
static float IRAM_ATTR clamp(float value, float minimum, float maximum) {
  /* Preserve values already inside the realizable actuator interval. */
  if (value < minimum) {
    return minimum;
  }
  if (value > maximum) {
    return maximum;
  }
  return value;
}

/**
 * @brief Check whether a complete PID configuration is usable.
 *
 * This private helper is called only by `esp_pid_init()`. Keeping validation in
 * one function prevents a partially valid configuration from being copied into
 * an instance.
 */
static bool IRAM_ATTR valid_config(const esp_pid_config_t *config) {
  /* Reject a missing configuration and every NaN or infinite scalar. */
  if (config == NULL || !isfinite(config->kp) || !isfinite(config->ki) ||
      !isfinite(config->kd) || !isfinite(config->output_min) ||
      !isfinite(config->output_max) ||
      !isfinite(config->derivative_filter_tau_s) ||
      !isfinite(config->anti_windup_tracking_time_s) ||
      !isfinite(config->maximum_dt_s)) {
    return false;
  }

  /* Check relationships and parameters that must never be negative. */
  if (config->output_min >= config->output_max ||
      config->derivative_filter_tau_s < 0.0f ||
      config->anti_windup_tracking_time_s < 0.0f ||
      config->maximum_dt_s < 0.0f) {
    return false;
  }

  /* Accept only derivative modes with a defined mathematical interpretation. */
  return config->derivative_source == ESP_PID_DERIVATIVE_ON_ERROR ||
         config->derivative_source == ESP_PID_DERIVATIVE_ON_MEASUREMENT;
}

esp_err_t IRAM_ATTR esp_pid_init(esp_pid_t *pid,
                                 const esp_pid_config_t *config) {
  /* Validate both the destination and the complete configuration first. */
  if (pid == NULL || !valid_config(config)) {
    return ESP_ERR_INVALID_ARG;
  }

  /* Initialize the whole object deterministically and copy the configuration.
   */
  *pid = (esp_pid_t){
      .config = *config,
      .initialized = true,
  };
  return ESP_OK;
}

esp_err_t IRAM_ATTR esp_pid_reset(esp_pid_t *pid) {
  /* A reset is meaningful only after a successful initialization. */
  if (pid == NULL || !pid->initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  /* Clear every dynamic and diagnostic field, preserving config and lifecycle.
   */
  pid->state = (esp_pid_state_t){0};
  return ESP_OK;
}

esp_err_t IRAM_ATTR esp_pid_update(esp_pid_t *pid, float reference,
                                   float measurement, float dt_s,
                                   float *output) {
  /* Validate the instance lifecycle and the required output destination. */
  if (pid == NULL || output == NULL || !pid->initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  /* Stop NaN or infinity from contaminating the persistent controller state. */
  if (!isfinite(reference) || !isfinite(measurement) || !isfinite(dt_s)) {
    return ESP_ERR_INVALID_ARG;
  }

  /* Use short aliases to make the equations below match standard PID notation.
   */
  const esp_pid_config_t *config = &pid->config;
  esp_pid_state_t *state = &pid->state;

  /*
   * Qualify the sample interval. A zero/negative or implausibly long interval
   * cannot safely be used in differentiation or integration. A configured
   * maximum of zero disables only the upper-bound test.
   */
  bool valid_dt = dt_s > 0.0f &&
                  (config->maximum_dt_s == 0.0f || dt_s < config->maximum_dt_s);
  state->last_dt_valid = valid_dt;

  /* Proportional block: P = Kp * (reference - measurement). */
  state->error = reference - measurement;
  state->proportional_term = config->kp * state->error;

  /*
   * Derivative and filter block. The first valid input establishes history but
   * deliberately produces no derivative impulse. Invalid dt holds the filter.
   */
  if (valid_dt && state->previous_input_valid) {
    /* Select either de/dt or dy/dt as the raw derivative signal. */
    float raw_derivative =
        config->derivative_source == ESP_PID_DERIVATIVE_ON_MEASUREMENT
            ? (measurement - state->previous_measurement) / dt_s
            : (state->error - state->previous_error) / dt_s;

    /* Apply a first-order low-pass; tau=0 passes the raw derivative directly.
     */
    float derivative_alpha =
        config->derivative_filter_tau_s > 0.0f
            ? dt_s / (config->derivative_filter_tau_s + dt_s)
            : 1.0f;
    state->filtered_derivative +=
        derivative_alpha * (raw_derivative - state->filtered_derivative);
  }

  /* Refresh sample history even when dt is invalid, avoiding a stale baseline.
   */
  state->previous_error = state->error;
  state->previous_measurement = measurement;
  state->previous_input_valid = true;

  /*
   * Derivative contribution. Differentiating the measurement requires a minus
   * sign because an increasing measurement reduces the control error.
   */
  float derivative_sign =
      config->derivative_source == ESP_PID_DERIVATIVE_ON_MEASUREMENT ? -1.0f
                                                                     : 1.0f;
  state->derivative_term =
      derivative_sign * config->kd * state->filtered_derivative;

  /* Integral block: first form the ordinary rectangular-rule candidate. */
  float candidate_integral = state->integral_term;
  if (valid_dt) {
    candidate_integral += config->ki * state->error * dt_s;
  }

  /*
   * Actuator model for anti-windup. Saturating the tentative PID sum gives the
   * output the actuator can realize during this sample.
   */
  float candidate_output =
      state->proportional_term + candidate_integral + state->derivative_term;
  float realizable_output =
      clamp(candidate_output, config->output_min, config->output_max);

  /*
   * Back-calculation block. The tracking error pulls the integrator toward a
   * value consistent with the saturated actuator. A zero tracking time turns
   * this correction off; invalid dt freezes all integral evolution.
   */
  if (valid_dt && config->anti_windup_tracking_time_s > 0.0f) {
    candidate_integral += (realizable_output - candidate_output) * dt_s /
                          config->anti_windup_tracking_time_s;
  }
  state->integral_term = candidate_integral;

  /* Recompose the PID after back-calculation and publish its bounded result. */
  state->unsaturated_output =
      state->proportional_term + state->integral_term + state->derivative_term;
  state->output =
      clamp(state->unsaturated_output, config->output_min, config->output_max);
  *output = state->output;
  return ESP_OK;
}

esp_err_t IRAM_ATTR esp_pid_get_state(const esp_pid_t *pid,
                                      esp_pid_state_t *state) {
  /* Require a live instance and a valid snapshot destination. */
  if (pid == NULL || state == NULL || !pid->initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  /* Return a by-value snapshot so the caller cannot mutate internal state. */
  *state = pid->state;
  return ESP_OK;
}
