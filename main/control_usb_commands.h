#ifndef CONTROL_USB_COMMANDS_H_
#define CONTROL_USB_COMMANDS_H_

#include "esp_timeseries_usb_transport.h"
#include "motor_controller.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief CONTROL command names appended to the transport HELP response.
 *
 * realtime_loop_start() concatenates this string with the CAL command help.
 */
#define CONTROL_USB_COMMAND_HELP                                               \
  "CONTROL_GET,CONTROL_SET_<kp>_<ki>_<kd>_<period_s>,CONTROL_DEFAULTS"

/**
 * @brief Non-owning link between the USB task and volatile controller settings.
 *
 * The pointed configuration has application lifetime. The USB task is its only
 * writer; the real-time task receives a snapshot through the ARM request path.
 */
typedef struct {
  /** Mutable RAM configuration used by the next successful closed-loop ARM. */
  motor_controller_config_t *configuration;
} control_usb_command_context_t;

/**
 * @brief Process volatile closed-loop parameter commands.
 *
 * CONTROL GET reports the configuration to be copied on the next successful
 * ARM. CONTROL SET replaces all four values after validation. CONTROL DEFAULTS
 * restores the firmware defaults. No command accesses NVS or changes an
 * experiment that has already been armed.
 *
 * This external callback is called by application_usb_command_handler() in the
 * USB transport task. It is not real-time safe and must not run in an ISR.
 *
 * @param command NUL-terminated line owned by the transport for this call.
 * @param io Non-null synchronous response interface; it is not retained.
 * @param context control_usb_command_context_t with application lifetime.
 * @return true after handling any CONTROL command, including malformed ones;
 * false for commands belonging to other namespaces or invalid callback input.
 */
bool control_usb_command_handler(const char *command,
                                 const esp_timeseries_usb_command_io_t *io,
                                 void *context);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_USB_COMMANDS_H_ */
