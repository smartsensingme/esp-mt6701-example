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
};

void app_main(void) {
  ESP_LOGI(TAG, "MT6701: 4 kHz acquisition/Kalman, 1 kHz PID control loop");

  esp_err_t nvs_error = nvs_flash_init();
  if (nvs_error == ESP_ERR_NVS_NO_FREE_PAGES ||
      nvs_error == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    nvs_error = nvs_flash_init();
  }
  ESP_ERROR_CHECK(nvs_error);

  /* MCPWM must be ready before the real-time task is allowed to command it. */
  ESP_LOGI(TAG, "Initializing H-Bridge motor driver...");
  if (engine_driver_init(&motor) != 0) {
    ESP_LOGE(TAG, "Failed to initialize H-Bridge motor driver");
    return;
  }

  /*
   * This creates the Core 0 logger and Core 1 real-time task. app_main() may
   * return after success because both tasks and the motor state outlive it.
   */
  esp_err_t err = realtime_loop_start(&motor);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Could not start real-time loop: %s", esp_err_to_name(err));
    /* Fail safe: never leave a nonzero command after partial startup failure.
     */
    engine_driver_set_speed(&motor, 0.0f);
  }
}
