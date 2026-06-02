#ifndef AS5600_H_
#define AS5600_H_

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#if CONFIG_AS5600_THREAD_SAFE
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#endif

#define AS5600_I2C_ADDRESS    0x36

// Registers
#define AS5600_REG_ZMCO        0x00
#define AS5600_REG_ZPOS_H      0x01
#define AS5600_REG_ZPOS_L      0x02
#define AS5600_REG_MPOS_H      0x03
#define AS5600_REG_MPOS_L      0x04
#define AS5600_REG_MANG_H      0x05
#define AS5600_REG_MANG_L      0x06
#define AS5600_REG_CONF_H      0x07
#define AS5600_REG_CONF_L      0x08
#define AS5600_REG_STATUS      0x0B
#define AS5600_REG_RAW_ANGLE_H 0x0C
#define AS5600_REG_RAW_ANGLE_L 0x0D
#define AS5600_REG_ANGLE_H     0x0E
#define AS5600_REG_ANGLE_L     0x0F

// CONF Register masks
#define AS5600_CONF_WD_MASK     0x80
#define AS5600_CONF_FTH_MASK    0x38
#define AS5600_CONF_SF_MASK     0x06
#define AS5600_CONF_PWMF_MASK   0x30
#define AS5600_CONF_OUTS_MASK   0x0C
#define AS5600_CONF_HYST_MASK   0x03

// STATUS Register flags
#define AS5600_STATUS_MD        0x20 // Magnet detected
#define AS5600_STATUS_ML        0x10 // Magnet too weak
#define AS5600_STATUS_MH        0x08 // Magnet too strong

typedef struct {
    uint8_t hysteresis;
    uint8_t slow_filter;
    uint8_t fast_filter_threshold;
} as5600_config_t;

typedef struct {
    i2c_master_dev_handle_t i2c_dev;
    as5600_config_t config;
#if CONFIG_AS5600_THREAD_SAFE
    SemaphoreHandle_t lock;
    StaticSemaphore_t lock_buffer;
#endif
} as5600_dev_t;

esp_err_t as5600_init(as5600_dev_t *dev, i2c_master_dev_handle_t i2c_dev, const as5600_config_t *config);
esp_err_t as5600_read_raw_angle(as5600_dev_t *dev, uint16_t *raw_angle);
esp_err_t as5600_read_angle(as5600_dev_t *dev, uint16_t *angle);
esp_err_t as5600_read_status(as5600_dev_t *dev, uint8_t *status);
esp_err_t as5600_read_angles_and_status(as5600_dev_t *dev, uint16_t *raw_angle, uint16_t *angle, uint8_t *status);

#endif // AS5600_H_
