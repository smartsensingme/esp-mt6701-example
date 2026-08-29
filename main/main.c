#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "engine_angle_kalman.h"
#include "engine_driver.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "kalman.h"
#include "mt6701.h"
#include <stdio.h>

#define I2C_PORT_NUM I2C_NUM_0

static const char *TAG = "APP_MAIN";

// Static configuration for H-Bridge engine driver using Kconfig settings
static struct engine_config motor = {
    .pin_fwd = CONFIG_ENGINE_PIN_RPWM,
    .pin_rev = CONFIG_ENGINE_PIN_LPWM,
    .pin_enable = CONFIG_ENGINE_PIN_ENABLE,
    .pwm_freq_hz = CONFIG_ENGINE_PWM_FREQ_HZ,
};

void app_main(void) {
  ESP_LOGI(TAG,
           "MT6701 Magnetic Encoder Demonstration - High Speed Raw "
           "Sampling, Multi-Turn, Velocity & 3D Kalman Filter (ESP-IDF v6)");

  // Initialize H-Bridge Engine Driver
  ESP_LOGI(TAG, "Initializing H-Bridge motor driver...");
  if (engine_driver_init(&motor) == 0) {
    ESP_LOGI(TAG, "H-Bridge motor driver successfully initialized. Setting "
                  "50%% speed Forward.");
    engine_driver_set_speed(&motor, 50.0f);
  } else {
    ESP_LOGE(TAG, "Failed to initialize H-Bridge motor driver!");
  }

  // 1. Initialize I2C Master Bus
  i2c_master_bus_config_t bus_config = {
      .i2c_port = I2C_PORT_NUM,
      .sda_io_num = CONFIG_APP_I2C_SDA_PIN,
      .scl_io_num = CONFIG_APP_I2C_SCL_PIN,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 7,
      .flags.enable_internal_pullup = true,
  };
  i2c_master_bus_handle_t bus_handle;
  ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

  // 2. Add MT6701 Device to the I2C Master Bus
  i2c_device_config_t dev_config = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = MT6701_I2C_ADDRESS,
      .scl_speed_hz = 400000,
  };
  i2c_master_dev_handle_t i2c_dev;
  ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_config, &i2c_dev));

  // 3. Initialize MT6701 driver
  mt6701_dev_t mt6701_dev;
  esp_err_t err = mt6701_init(&mt6701_dev, i2c_dev);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "MT6701 Driver Init failed: %s", esp_err_to_name(err));
    return;
  }

  // Set software configs if needed (defaults to offset 0 and CW direction)
  mt6701_set_software_direction(&mt6701_dev, MT6701_DIR_CW);

  // 4. Initialize Kalman Filter 3D
  struct kalman_3d filter;
  float initial_deg = 0.0f;

  if (mt6701_get_last_angle_degrees(&mt6701_dev, &initial_deg) == ESP_OK) {
    ESP_LOGI(TAG, "Initial calibrated angle: %.3f deg", initial_deg);
  } else {
    ESP_LOGW(
        TAG,
        "Could not fetch initial angle, starting Kalman Filter at 0.0 deg");
  }

  /*
   * Configuração customizada para o Filtro de Kalman 3D adaptada para o MT6701:
   *
   * - r (Covariância do ruído de medição):
   *   O MT6701 possui ruído de transição RMS típico de 0.01° (conforme
   * datasheet), sendo cerca de 10x mais preciso e menos ruidoso que o AS5600.
   *   Ajustamos r para 0.0004f (equivalente a uma desviação padrão de 0.02°),
   *   fazendo o filtro confiar muito mais na medição direta e diminuindo o lag.
   */
  kalman_3d_config_t filter_cfg = {
      .q_theta = 0.001f,
      .q_omega = 10.0f,
      .q_alpha = 100.0f,
      .r = 0.0004f,
  };

  kalman_3d_init(&filter, initial_deg, &filter_cfg);

  uint32_t loop_count = 0;
  TickType_t last_wake_time = xTaskGetTickCount();
  int64_t last_time = esp_timer_get_time();

  while (1) {
    // Calcula o dt real entre duas amostragens.
    int64_t now = esp_timer_get_time();
    float dt = (float)(now - last_time) /
               1000000.0f; // Convert microseconds to seconds
    last_time = now;

    // Fetch angle at 1 kHz (every 1 ms) and update software tracking
    if (mt6701_update(&mt6701_dev) == ESP_OK) {
      float measured_deg;
      if (mt6701_get_last_angle_degrees(&mt6701_dev, &measured_deg) == ESP_OK &&
          dt > 0.0f && dt < 0.1f) { // Protection against scheduling anomalies
        engine_angle_kalman_3d_update(&filter, measured_deg, dt);
      }
    }

    // Print software turns and velocity every 2 seconds (2000 ticks of 1 ms)
    if (loop_count % 2000 == 0) {
      int32_t turns = 0;
      float velocity = 0.0f;
      mt6701_get_total_turns(&mt6701_dev, &turns);
      mt6701_get_velocity(&mt6701_dev, &velocity);
      printf(
          "[MT6701 State] Total Turns: %ld | Filtered Velocity: %.3f rad/s\n",
          turns, velocity);
    }

    // Print estimated state variables every 200 ms (200 ticks of 1 ms)
    if (loop_count % 200 == 0) {
      float measured_deg = 0.0f;
      mt6701_get_last_angle_degrees(&mt6701_dev, &measured_deg);
      float est_angle = filter.x[0];
      float est_speed_dps = filter.x[1];
      float est_speed_rpm = est_speed_dps / 6.0f; // dps to RPM
      float est_accel_dps2 = filter.x[2];
      float est_accel_rpm_s = est_accel_dps2 / 6.0f; // dps/s^2 to RPM/s

      printf("Measured Angle: %.3f deg | Kalman Angle: %.3f deg | Speed: %.3f "
             "RPM | "
             "Accel: %.3f RPM/s | dt: %.6f s\n",
             measured_deg, est_angle, est_speed_rpm, est_accel_rpm_s, dt);
    }

    loop_count++;
    // Delay exactly 1 ms relative to last_wake_time to achieve constant 1 kHz
    // sampling rate
    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(1));
  }
}
