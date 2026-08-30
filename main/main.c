#include "engine_driver.h"
#include "esp_log.h"
#include "realtime_loop.h"

static const char *TAG = "APP_MAIN";

static struct engine_config motor = {
    .pin_fwd = CONFIG_ENGINE_PIN_RPWM,
    .pin_rev = CONFIG_ENGINE_PIN_LPWM,
    .pin_enable = CONFIG_ENGINE_PIN_ENABLE,
    .pwm_freq_hz = CONFIG_ENGINE_PWM_FREQ_HZ,
};

void app_main(void) {
  ESP_LOGI(TAG, "MT6701: 4 kHz acquisition/Kalman, 1 kHz dummy control loop");

  ESP_LOGI(TAG, "Initializing H-Bridge motor driver...");
  if (engine_driver_init(&motor) != 0) {
    ESP_LOGE(TAG, "Failed to initialize H-Bridge motor driver");
    return;
  }

  esp_err_t err = realtime_loop_start(&motor);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Could not start real-time loop: %s", esp_err_to_name(err));
    engine_driver_set_speed(&motor, 0.0f);
  }
}
