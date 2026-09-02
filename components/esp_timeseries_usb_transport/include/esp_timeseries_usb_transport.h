#ifndef ESP_TIMESERIES_USB_TRANSPORT_H_
#define ESP_TIMESERIES_USB_TRANSPORT_H_

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef esp_err_t (*esp_timeseries_usb_arm_handler_t)(uint32_t sample_rate_hz,
                                                      bool calibration_mode,
                                                      void *context);

typedef struct {
  esp_timeseries_usb_arm_handler_t arm_handler;
  void *arm_handler_context;
} esp_timeseries_usb_transport_config_t;

/** Install the USB driver and create the Core 0 command task. */
esp_err_t esp_timeseries_usb_transport_start(
    const esp_timeseries_usb_transport_config_t *config);

/** Stop the command task and uninstall the driver from the calling core. */
void esp_timeseries_usb_transport_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* ESP_TIMESERIES_USB_TRANSPORT_H_ */
