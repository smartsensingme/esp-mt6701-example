#ifndef REALTIME_LOOP_H_
#define REALTIME_LOOP_H_

#include "engine_driver.h"
#include "esp_err.h"

/** Mean MT6701 acquisition and Kalman update rate in hertz. */
#define REALTIME_SENSOR_RATE_HZ 3000U
/** PID and motor-command update rate in hertz. */
#define REALTIME_CONTROL_RATE_HZ 1000U

/**
 * @brief Start the complete dual-rate application runtime.
 *
 * Creates the control task and, when diagnostics are enabled, the logger:
 * - Core 1: MT6701 acquisition and Kalman at 3 kHz, control at 1 kHz, and
 *   timing instrumentation. This task owns the sensor and motor after startup.
 * - Core 0: low-priority telemetry logger. It receives copied snapshots and
 *   never accesses the live estimator or controller state.
 *
 * The MT6701 I2C bus and GPTimer are created by the Core 1 task so peripheral
 * interrupt allocation follows the real-time task affinity.
 *
 * This public application entry point is called only by app_main(). On success
 * it retains the motor pointer and returns after creating the long-lived tasks.
 * It must be called once, from task context, after engine_driver_init().
 *
 * @param motor Initialized motor driver configuration. It must remain valid
 * throughout the application lifetime.
 * @return ESP_OK when all enabled resources were created;
 * ESP_ERR_INVALID_ARG for a null motor; ESP_ERR_NO_MEM when the control task
 * cannot be created; or an initialization error propagated by the LUT,
 * recorder, USB transport, or telemetry modules.
 */
esp_err_t realtime_loop_start(struct engine_config *motor);

#endif /* REALTIME_LOOP_H_ */
