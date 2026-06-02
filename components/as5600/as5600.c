#include "as5600.h"
#include "esp_log.h"

static const char *TAG = "AS5600";

static esp_err_t as5600_reg_read(as5600_dev_t *dev, uint8_t reg, uint8_t *val) {
    return i2c_master_transmit_receive(dev->i2c_dev, &reg, 1, val, 1, -1);
}

static esp_err_t as5600_reg_write(as5600_dev_t *dev, uint8_t reg, uint8_t val) {
    uint8_t buffer[2] = { reg, val };
    return i2c_master_transmit(dev->i2c_dev, buffer, 2, -1);
}

esp_err_t as5600_init(as5600_dev_t *dev, i2c_master_dev_handle_t i2c_dev, const as5600_config_t *config) {
    if (!dev || !i2c_dev || !config) {
        return ESP_ERR_INVALID_ARG;
    }

    dev->i2c_dev = i2c_dev;
    dev->config = *config;

#if CONFIG_AS5600_THREAD_SAFE
    dev->lock = xSemaphoreCreateMutexStatic(&dev->lock_buffer);
    if (dev->lock == NULL) {
        ESP_LOGE(TAG, "Failed to create static mutex");
        return ESP_ERR_NO_MEM;
    }
#endif

#if CONFIG_AS5600_THREAD_SAFE
    if (xSemaphoreTake(dev->lock, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
#endif

    esp_err_t ret;
    uint8_t conf_h, conf_l;
    uint8_t sf_bits;

    // Verify device presence by reading status register
    uint8_t status;
    ret = as5600_reg_read(dev, AS5600_REG_STATUS, &status);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to communicate with AS5600 (err: %d)", ret);
        goto out;
    }

    // Read current configuration
    ret = as5600_reg_read(dev, AS5600_REG_CONF_H, &conf_h);
    if (ret != ESP_OK) goto out;

    ret = as5600_reg_read(dev, AS5600_REG_CONF_L, &conf_l);
    if (ret != ESP_OK) goto out;

    // Map slow filter factor (16, 8, 4, 2) to register bits (00, 01, 10, 11)
    switch (config->slow_filter) {
        case 16: sf_bits = 0x00; break;
        case 8:  sf_bits = 0x01; break;
        case 4:  sf_bits = 0x02; break;
        case 2:  sf_bits = 0x03; break;
        default: sf_bits = 0x00; break;
    }

    // Apply configurations to CONF_H: Fast Filter Threshold & Slow Filter
    conf_h &= ~(AS5600_CONF_FTH_MASK | AS5600_CONF_SF_MASK);
    conf_h |= ((config->fast_filter_threshold << 3) & AS5600_CONF_FTH_MASK);
    conf_h |= ((sf_bits << 1) & AS5600_CONF_SF_MASK);

    // Apply configurations to CONF_L: Hysteresis
    conf_l &= ~AS5600_CONF_HYST_MASK;
    conf_l |= (config->hysteresis & AS5600_CONF_HYST_MASK);

    // Write configurations back
    ret = as5600_reg_write(dev, AS5600_REG_CONF_H, conf_h);
    if (ret != ESP_OK) goto out;

    ret = as5600_reg_write(dev, AS5600_REG_CONF_L, conf_l);
    if (ret != ESP_OK) goto out;

    ESP_LOGI(TAG, "Initialized successfully (Hysteresis: %d, Slow Filter: %dx, Fast Filter Threshold: %d)",
             config->hysteresis, config->slow_filter, config->fast_filter_threshold);

out:
#if CONFIG_AS5600_THREAD_SAFE
    xSemaphoreGive(dev->lock);
#endif
    return ret;
}

esp_err_t as5600_read_raw_angle(as5600_dev_t *dev, uint16_t *raw_angle) {
    if (!dev || !raw_angle) {
        return ESP_ERR_INVALID_ARG;
    }

#if CONFIG_AS5600_THREAD_SAFE
    if (xSemaphoreTake(dev->lock, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
#endif

    uint8_t reg = AS5600_REG_RAW_ANGLE_H;
    uint8_t buffer[2];
    esp_err_t err = i2c_master_transmit_receive(dev->i2c_dev, &reg, 1, buffer, 2, -1);
    if (err == ESP_OK) {
        *raw_angle = (((uint16_t)buffer[0] << 8) | buffer[1]) & 0x0FFF;
    }

#if CONFIG_AS5600_THREAD_SAFE
    xSemaphoreGive(dev->lock);
#endif
    return err;
}

esp_err_t as5600_read_angle(as5600_dev_t *dev, uint16_t *angle) {
    if (!dev || !angle) {
        return ESP_ERR_INVALID_ARG;
    }

#if CONFIG_AS5600_THREAD_SAFE
    if (xSemaphoreTake(dev->lock, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
#endif

    uint8_t reg = AS5600_REG_ANGLE_H;
    uint8_t buffer[2];
    esp_err_t err = i2c_master_transmit_receive(dev->i2c_dev, &reg, 1, buffer, 2, -1);
    if (err == ESP_OK) {
        *angle = (((uint16_t)buffer[0] << 8) | buffer[1]) & 0x0FFF;
    }

#if CONFIG_AS5600_THREAD_SAFE
    xSemaphoreGive(dev->lock);
#endif
    return err;
}

esp_err_t as5600_read_status(as5600_dev_t *dev, uint8_t *status) {
    if (!dev || !status) {
        return ESP_ERR_INVALID_ARG;
    }

#if CONFIG_AS5600_THREAD_SAFE
    if (xSemaphoreTake(dev->lock, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
#endif

    uint8_t reg = AS5600_REG_STATUS;
    esp_err_t err = i2c_master_transmit_receive(dev->i2c_dev, &reg, 1, status, 1, -1);

#if CONFIG_AS5600_THREAD_SAFE
    xSemaphoreGive(dev->lock);
#endif
    return err;
}

esp_err_t as5600_read_angles_and_status(as5600_dev_t *dev, uint16_t *raw_angle, uint16_t *angle, uint8_t *status) {
    if (!dev) {
        return ESP_ERR_INVALID_ARG;
    }

#if CONFIG_AS5600_THREAD_SAFE
    if (xSemaphoreTake(dev->lock, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
#endif

    uint8_t reg = AS5600_REG_STATUS;
    uint8_t buffer[5];
    esp_err_t err = i2c_master_transmit_receive(dev->i2c_dev, &reg, 1, buffer, 5, -1);
    if (err == ESP_OK) {
        if (status) {
            *status = buffer[0];
        }
        if (raw_angle) {
            *raw_angle = (((uint16_t)buffer[1] << 8) | buffer[2]) & 0x0FFF;
        }
        if (angle) {
            *angle = (((uint16_t)buffer[3] << 8) | buffer[4]) & 0x0FFF;
        }
    }

#if CONFIG_AS5600_THREAD_SAFE
    xSemaphoreGive(dev->lock);
#endif
    return err;
}
