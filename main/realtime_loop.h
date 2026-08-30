#ifndef REALTIME_LOOP_H_
#define REALTIME_LOOP_H_

#include "engine_driver.h"
#include "esp_err.h"

#define REALTIME_SENSOR_RATE_HZ 4000U
#define REALTIME_CONTROL_RATE_HZ 1000U

/**
 * Start the Core 1 sensor/estimator/controller task and the Core 0 logger.
 * The MT6701 I2C bus and GPTimer are created by the Core 1 task so their
 * interrupt allocation follows the real-time task affinity.
 */
esp_err_t realtime_loop_start(struct engine_config *motor);

#endif /* REALTIME_LOOP_H_ */
