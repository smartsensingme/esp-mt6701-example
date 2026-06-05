#include "as5600.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "engine_driver.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "kalman.h"
#include <stdio.h>

#define I2C_PORT_NUM I2C_NUM_0
#define AS5600_DIR_PIN                                                         \
  GPIO_NUM_4 // Pino para controle de direção (DIR) do AS5600

static const char *TAG = "APP_MAIN";

// Specialized 3D Kalman Filter update with wrap-around handling (360 -> 0
// degrees)
static void engine_angle_kalman_3d_update(struct kalman_3d *k,
                                          float measured_angle, float dt) {
  /* --- 1. PREDICT STEP --- */
  float h = 0.5f * dt * dt;

  /* x_pred = F * x */
  float x_pred_theta = k->x[0] + k->x[1] * dt + k->x[2] * h;
  float x_pred_omega = k->x[1] + k->x[2] * dt;
  float x_pred_alpha = k->x[2];

  /* P_pred = F * P * F^T + Q */
  float a0 = k->P[0][0] + dt * k->P[1][0] + h * k->P[2][0];
  float a1 = k->P[0][1] + dt * k->P[1][1] + h * k->P[2][1];
  float a2 = k->P[0][2] + dt * k->P[1][2] + h * k->P[2][2];

  float a4 = k->P[1][1] + dt * k->P[2][1];
  float a5 = k->P[1][2] + dt * k->P[2][2];

  float P_pred_00 = a0 + dt * a1 + h * a2 + k->Q_theta;
  float P_pred_01 = a1 + dt * a2;
  float P_pred_02 = a2;

  float P_pred_10 = P_pred_01;
  float P_pred_11 = a4 + dt * a5 + k->Q_omega;
  float P_pred_12 = a5;

  float P_pred_20 = P_pred_02;
  float P_pred_22 = k->P[2][2] + k->Q_alpha;

  /* --- 2. UPDATE / CORRECT STEP --- */
  /* Innovation (measurement error) */
  float y = measured_angle - x_pred_theta;

  /* Normalize y to [-180, 180] degrees to handle wrap-around from 360 to 0 */
  while (y > 180.0f) {
    y -= 360.0f;
  }
  while (y < -180.0f) {
    y += 360.0f;
  }

  /* Innovation Covariance: S = H * P_pred * H^T + R */
  float S = P_pred_00 + k->R;

  /* Kalman Gain: K = P_pred * H^T * S^-1 */
  float K_0 = P_pred_00 / S;
  float K_1 = P_pred_10 / S;
  float K_2 = P_pred_20 / S;

  /* Correct State: x = x_pred + K * y */
  k->x[0] = x_pred_theta + K_0 * y;
  k->x[1] = x_pred_omega + K_1 * y;
  k->x[2] = x_pred_alpha + K_2 * y;

  /* Keep estimated angle within [0, 360) range */
  while (k->x[0] >= 360.0f) {
    k->x[0] -= 360.0f;
  }
  while (k->x[0] < 0.0f) {
    k->x[0] += 360.0f;
  }

  /* Correct Covariance: P = (I - K * H) * P_pred */
  k->P[0][0] = (1.0f - K_0) * P_pred_00;
  k->P[0][1] = (1.0f - K_0) * P_pred_01;
  k->P[0][2] = (1.0f - K_0) * P_pred_02;

  k->P[1][0] = k->P[0][1];
  k->P[1][1] = P_pred_11 - K_1 * P_pred_01;
  k->P[1][2] = P_pred_12 - K_1 * P_pred_02;

  k->P[2][0] = k->P[0][2];
  k->P[2][1] = k->P[1][2];
  k->P[2][2] = P_pred_22 - K_2 * P_pred_02;
}

// Convert 12-bit register value (0..4095) to float degrees (0.0..359.9)
static float raw_to_degrees(uint16_t raw) {
  return ((float)raw * 360.0f) / 4096.0f;
}

// Static configuration for H-Bridge engine driver using Kconfig settings
static struct engine_config motor = {
    .pin_fwd = CONFIG_ENGINE_PIN_RPWM,
    .pin_rev = CONFIG_ENGINE_PIN_LPWM,
    .pin_enable = CONFIG_ENGINE_PIN_ENABLE,
    .pwm_freq_hz = CONFIG_ENGINE_PWM_FREQ_HZ,
};

void app_main(void) {
  // Configuração do pino DIR do AS5600 (GPIO 4) para nível lógico HIGH (3.3V)
  // Isso atende ao requisito de manter o pino DIR em nível alto (sinal de 5V ou
  // compatível) para definir a direção.
  ESP_LOGI(
      TAG,
      "Configurando GPIO %d como saída em nível HIGH para o DIR do AS5600...",
      AS5600_DIR_PIN);
  gpio_config_t io_conf = {
      .pin_bit_mask = (1ULL << AS5600_DIR_PIN),
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  ESP_ERROR_CHECK(gpio_config(&io_conf));
  ESP_ERROR_CHECK(gpio_set_level(AS5600_DIR_PIN, 1));
  ESP_LOGI(TAG, "GPIO %d configurado com sucesso e definido como HIGH",
           AS5600_DIR_PIN);

  ESP_LOGI(TAG, "AS5600 Magnetic Encoder Demonstration - High Speed Raw "
                "Sampling & 3D Kalman Filter (ESP-IDF v6)");

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

  // 2. Add AS5600 Device to the I2C Master Bus
  i2c_device_config_t dev_config = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = AS5600_I2C_ADDRESS,
      .scl_speed_hz = 400000,
  };
  i2c_master_dev_handle_t i2c_dev;
  ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_config, &i2c_dev));

  // 3. Initialize AS5600 driver
  as5600_config_t as5600_cfg = {
      .hysteresis = 0,            // 0 LSB
      .slow_filter = 2,           // 2x slow filter
      .fast_filter_threshold = 2, // 2 LSB fast filter threshold
  };
  as5600_dev_t as5600_dev;
  esp_err_t err = as5600_init(&as5600_dev, i2c_dev, &as5600_cfg);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "AS5600 Driver Init failed: %s", esp_err_to_name(err));
    return;
  }

  // 4. Initialize Kalman Filter 3D
  struct kalman_3d filter;
  uint16_t initial_raw_angle = 0;
  float initial_deg = 0.0f;

  if (as5600_read_raw_angle(&as5600_dev, &initial_raw_angle) == ESP_OK) {
    initial_deg = raw_to_degrees(initial_raw_angle);
    ESP_LOGI(TAG, "Initial raw angle: %.3f deg", initial_deg);
  } else {
    ESP_LOGW(
        TAG,
        "Could not fetch initial angle, starting Kalman Filter at 0.0 deg");
  }

  /*
   * Configuração customizada para o Filtro de Kalman 3D:
   *
   * - q_theta (Covariância do ruído do processo para posição/ângulo):
   *   Valores menores fazem o filtro confiar mais no modelo físico (mais suave,
   * mas com maior atraso/lag). Valores maiores fazem o filtro confiar mais nas
   * medições (mais responsivo, mas com mais ruído/jitter). Valor padrão:
   * 0.001f.
   *
   * - q_omega (Covariância do ruído do processo para velocidade angular):
   *   Determina quão rápido o filtro reage a mudanças na velocidade angular.
   *   Valores maiores permitem adaptação mais rápida a
   * acelerações/desacelerações repentinas. Valor padrão: 10.0f.
   *
   * - q_alpha (Covariância do ruído do processo para aceleração angular):
   *   Determina a velocidade com que o filtro rastreia mudanças na aceleração.
   *   Valor padrão: 100.0f.
   *
   * - r (Covariância do ruído de medição):
   *   Representa o nível de ruído/variância nas leituras brutas do sensor
   * AS5600. Um valor maior indica que o sensor é mais ruidoso, fazendo o filtro
   * suavizar mais os dados. Valor padrão: 0.018f.
   *
   * Guia de Ajuste (Tuning):
   * 1. Se o ângulo filtrado apresentar atraso (lag) durante a rotação física,
   * aumente q_theta, q_omega e q_alpha.
   * 2. Se o ângulo filtrado oscilar muito (ruído/jitter) quando o sensor
   * estiver parado ou em movimento lento, diminua q_theta/q_omega ou aumente r.
   */
  kalman_3d_config_t filter_cfg = {
      .q_theta = 0.001f,
      .q_omega = 10.0f,
      .q_alpha = 100.0f,
      .r = 0.018f,
  };

  kalman_3d_init(&filter, initial_deg, &filter_cfg);

  uint32_t loop_count = 0;
  TickType_t last_wake_time = xTaskGetTickCount();
  int64_t last_time = esp_timer_get_time();

  while (1) {
    uint16_t raw_val = 0;

    // Calcula o dt real entre duas amostragens. Você tem que garantir que esse
    // dt é bem conhecido para o filtro de Kalman funcionar corretamente.
    int64_t now = esp_timer_get_time();
    float dt = (float)(now - last_time) /
               1000000.0f; // Convert microseconds to seconds
    last_time = now;

    // Fetch RAW ANGLE at 1 kHz (every 1 ms)
    // Only 2 bytes read, maximizes rate and avoids auto-increment bug on other
    // registers
    if (as5600_read_raw_angle(&as5600_dev, &raw_val) == ESP_OK) {
      float raw_deg = raw_to_degrees(raw_val);
      // Pass the measured dt to the Kalman filter
      if (dt > 0.0f &&
          dt <
              0.1f) { // Protection against startup jump or scheduling anomalies
        engine_angle_kalman_3d_update(&filter, raw_deg, dt);
      }
    }

    // Print sensor status every 2 seconds (2000 ticks of 1 ms)
    if (loop_count % 2000 == 0) {
      uint8_t status = 0;
      if (as5600_read_status(&as5600_dev, &status) == ESP_OK) {
        bool md = (status & AS5600_STATUS_MD) != 0;
        bool ml = (status & AS5600_STATUS_ML) != 0;
        bool mh = (status & AS5600_STATUS_MH) != 0;
        printf("[AS5600 Status] Byte: 0x%02X | Magnet: %s | Strength: %s\n",
               status, md ? "DETECTED" : "NOT DETECTED",
               ml ? "TOO WEAK" : (mh ? "TOO STRONG" : "OK"));
      } else {
        printf("[AS5600 Status] Failed to read status\n");
      }
    }

    // Print estimated state variables every 200 ms (200 ticks of 1 ms)
    if (loop_count % 200 == 0) {
      float raw_deg = raw_to_degrees(raw_val);
      float est_angle = filter.x[0];
      float est_speed_dps = filter.x[1];
      float est_speed_rpm = est_speed_dps / 6.0f; // dps to RPM
      float est_accel_dps2 = filter.x[2];
      float est_accel_rpm_s = est_accel_dps2 / 6.0f; // dps/s^2 to RPM/s

      printf("Raw Angle: %.3f deg | Kalman Angle: %.3f deg | Speed: %.3f RPM | "
             "Accel: %.3f RPM/s | dt: %.6f s\n",
             raw_deg, est_angle, est_speed_rpm, est_accel_rpm_s, dt);
    }

    loop_count++;
    // Delay exactly 1 ms relative to last_wake_time to achieve constant 1 kHz
    // sampling rate
    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(1));
  }
}
