#ifndef ANGLE_LUT_USB_COMMANDS_H_
#define ANGLE_LUT_USB_COMMANDS_H_

#include "esp_timeseries_usb_transport.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief CAL command names appended to the generic transport HELP response.
 *
 * The application passes this compile-time string to
 * esp_timeseries_usb_transport_start(); it owns no storage at runtime.
 */
#define ANGLE_LUT_USB_COMMAND_HELP                                             \
  "CAL_START_<hz>,CAL_STATUS,CAL_WRITE,CAL_READ,CAL_ENABLE,CAL_DISABLE,"       \
  "CAL_CLEAR"

/**
 * @brief Application callbacks required to connect CAL START to acquisition.
 *
 * The structure is configured by realtime_loop_start() and remains valid for
 * the lifetime of the USB transport task. It does not own either pointer.
 */
typedef struct {
  /** Callback that arms the recorder and selects the open-loop profile. */
  esp_timeseries_usb_arm_handler_t calibration_arm_handler;
  /** Opaque pointer passed unchanged to calibration_arm_handler. */
  void *calibration_arm_context;
} angle_lut_usb_command_context_t;

/**
 * @brief Process the CAL command family as an application protocol extension.
 *
 * This external callback matches esp_timeseries_usb_command_handler_t. In this
 * application it is called by application_usb_command_handler(), which is
 * called by the USB transport task. It returns false without performing I/O
 * for commands outside the CAL namespace. Recognized or malformed CAL commands
 * are answered here and return true.
 *
 * CAL START invokes the callback supplied in angle_lut_usb_command_context_t.
 * CAL WRITE blocks the USB task while receiving, validating, and persisting a
 * complete little-endian int16 LUT. It must never be called from the real-time
 * loop or an ISR.
 *
 * @param command NUL-terminated command line owned by the transport and valid
 * only for this call.
 * @param io Non-null transport operations used synchronously for replies and
 * binary transfer; the function does not retain the pointer.
 * @param context Optional angle_lut_usb_command_context_t supplied when the
 * transport was started; required only by CAL START.
 * @return true when the command belongs to the CAL namespace, including
 * malformed CAL commands that received an error; false otherwise.
 */
bool angle_lut_usb_command_handler(const char *command,
                                   const esp_timeseries_usb_command_io_t *io,
                                   void *context);

#ifdef __cplusplus
}
#endif

#endif /* ANGLE_LUT_USB_COMMANDS_H_ */
