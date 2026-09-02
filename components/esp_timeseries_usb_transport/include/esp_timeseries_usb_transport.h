#ifndef ESP_TIMESERIES_USB_TRANSPORT_H_
#define ESP_TIMESERIES_USB_TRANSPORT_H_

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Install the USB Serial/JTAG driver and create the Core 0 command task. */
esp_err_t esp_timeseries_usb_transport_start(void);

/** Stop the command task and uninstall the driver from the calling core. */
void esp_timeseries_usb_transport_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* ESP_TIMESERIES_USB_TRANSPORT_H_ */
