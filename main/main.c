#include "engine_driver.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "realtime_loop.h"

static const char *TAG = "APP_MAIN";

/*
 * Static storage is intentional: realtime_loop_start() keeps this pointer after
 * app_main() returns. Pin numbers and PWM frequency come from menuconfig.
 */
static struct engine_config motor = {
    .pin_fwd = CONFIG_ENGINE_PIN_RPWM,
    .pin_rev = CONFIG_ENGINE_PIN_LPWM,
    .pin_enable = CONFIG_ENGINE_PIN_ENABLE,
    .pwm_freq_hz = CONFIG_ENGINE_PWM_FREQ_HZ,
    .direction_dead_time_us = CONFIG_ENGINE_DIRECTION_DEAD_TIME_US,
};

/**
 * @brief Initialize persistent services, the bridge, and application tasks.
 *
 * ESP-IDF calls this external entry point once from its main task. It
 * initializes NVS for the angle LUT, initializes the static motor instance, and
 * calls realtime_loop_start(). The function may return after successful task
 * creation because all retained state has static or application lifetime.
 */
void app_main(void) {
  /* Identification block: state the compiled acquisition/control topology. */
  ESP_LOGI(TAG, "MT6701: 3 kHz acquisition/Kalman, 1 kHz PID control loop");

  /* NVS block: recover the two ESP-IDF conditions that require partition erase
   * before retrying. Other failures remain fatal through ESP_ERROR_CHECK. */
  esp_err_t nvs_error = nvs_flash_init();
  if (nvs_error == ESP_ERR_NVS_NO_FREE_PAGES ||
      nvs_error == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    nvs_error = nvs_flash_init();
  }
  ESP_ERROR_CHECK(nvs_error);

  /* Actuator block: MCPWM must exist before Core 1 may command the bridge. */
  ESP_LOGI(TAG, "Initializing H-Bridge motor driver...");
  if (engine_driver_init(&motor) != 0) {
    ESP_LOGE(TAG, "Failed to initialize H-Bridge motor driver");
    return;
  }

  /* Runtime block: create the Core 0 reporter and Core 1 deterministic task. */
  esp_err_t err = realtime_loop_start(&motor);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Could not start real-time loop: %s", esp_err_to_name(err));
    /* Failure block: never retain drive after a partial runtime startup. */
    engine_driver_set_speed(&motor, 0.0f);
  }
}
