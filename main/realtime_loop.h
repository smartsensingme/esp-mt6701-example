#ifndef REALTIME_LOOP_H_
#define REALTIME_LOOP_H_

#include "engine_driver.h"
#include "esp_err.h"

#define REALTIME_SENSOR_RATE_HZ 4000U
#define REALTIME_CONTROL_RATE_HZ 1000U

/**
 * @brief Start the complete dual-rate application runtime.
 *
 * Creates two tasks:
 * - Core 1: MT6701 acquisition and Kalman at 4 kHz, control at 1 kHz, and
 *   timing instrumentation. This task owns the sensor and motor after startup.
 * - Core 0: low-priority telemetry logger. It receives copied snapshots and
 *   never accesses the live estimator or controller state.
 *
 * The MT6701 I2C bus and GPTimer are created by the Core 1 task so peripheral
 * interrupt allocation follows the real-time task affinity.
 *
 * @param motor Initialized motor driver configuration. It must remain valid
 * throughout the application lifetime.
 * @return ESP_OK when both tasks and the telemetry queue were created;
 * ESP_ERR_INVALID_ARG for a null motor; ESP_ERR_NO_MEM on allocation failure.
 */
esp_err_t realtime_loop_start(struct engine_config *motor);

#endif /* REALTIME_LOOP_H_ */
