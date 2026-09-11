/**
 * @file esp_timeseries_usb_transport.h
 * @brief Native USB Serial/JTAG command and binary-capture transport.
 *
 * The core protocol controls esp_timeseries_recorder without depending on a
 * sensor or control application. Optional synchronous callbacks let the
 * application coordinate ARM and implement additional commands.
 */
#ifndef ESP_TIMESERIES_USB_TRANSPORT_H_
#define ESP_TIMESERIES_USB_TRANSPORT_H_

#include "esp_err.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Synchronous I/O operations available inside a command callback.
 *
 * The transport passes a temporary pointer to this table. A callback may use it
 * before returning, but must not retain it or use it from another task. All
 * operations may block for the configured USB timeout.
 */
typedef struct {
  /** Send exactly length bytes, retrying partial writes until done or timeout.
   */
  esp_err_t (*write_all)(const void *data, size_t length);
  /** Receive exactly length bytes, retrying partial reads until done or
   * timeout. */
  esp_err_t (*read_all)(void *data, size_t length);
  /** Format and send text that fits in the transport response-line buffer. */
  esp_err_t (*sendf)(const char *format, ...);
  /** Send `ERR command=... code=... message=...` using the standard format. */
  void (*send_error)(const char *command, esp_err_t error, const char *message);
  /** Send current recorder status as a standard OK response. */
  void (*send_recorder_status)(const char *command);
} esp_timeseries_usb_command_io_t;

/**
 * @brief Optional application coordinator for the core ARM command.
 *
 * Called internally by the transport task through arm_capture(). It may prepare
 * application state before calling esp_timeseries_arm(). It must not send a
 * response; the transport converts the returned esp_err_t into the ARM reply.
 *
 * @param sample_rate_hz Rate parsed from the host command.
 * @param context arm_handler_context supplied at startup.
 * @return ESP_OK on success or an esp_err_t describing rejection.
 */
typedef esp_err_t (*esp_timeseries_usb_arm_handler_t)(uint32_t sample_rate_hz,
                                                      void *context);

/**
 * @brief Optional handler for commands outside the recorder core protocol.
 *
 * Called internally by process_command() in the USB transport task. Return true
 * only after recognizing the command and sending any required response. Return
 * false without performing I/O to request the standard UNKNOWN response.
 *
 * @param command NUL-terminated line without CR/LF; valid only during the call.
 * @param io Temporary synchronous I/O table; never retain or delegate it.
 * @param context command_handler_context supplied at startup.
 * @return true if handled, otherwise false.
 */
typedef bool (*esp_timeseries_usb_command_handler_t)(
    const char *command, const esp_timeseries_usb_command_io_t *io,
    void *context);

/** Startup configuration for optional application integration. */
typedef struct {
  /** ARM coordinator; NULL makes the transport call esp_timeseries_arm(). */
  esp_timeseries_usb_arm_handler_t arm_handler;
  /** Opaque persistent value passed to arm_handler. */
  void *arm_handler_context;
  /** Application command extension, or NULL for core commands only. */
  esp_timeseries_usb_command_handler_t command_handler;
  /** Opaque persistent value passed to command_handler. */
  void *command_handler_context;
  /** Persistent comma-separated HELP suffix; must not contain CR or LF. */
  const char *extension_help;
} esp_timeseries_usb_transport_config_t;

/**
 * @brief Install native USB Serial/JTAG and start the Core 0 command task.
 *
 * Called externally after esp_timeseries_init(). No component function calls it
 * internally. The configuration is shallow-copied, so callback contexts and
 * extension_help storage must remain valid until stop. Passing NULL enables the
 * core protocol with no application callbacks. When the component is disabled
 * in Kconfig this function is a successful no-op.
 *
 * @param config Optional persistent integration configuration.
 * @return ESP_OK; ESP_ERR_INVALID_ARG for unsafe help text;
 *         ESP_ERR_INVALID_STATE if the task/driver already exists;
 *         ESP_ERR_NO_MEM if task creation fails; or a USB driver error.
 */
esp_err_t esp_timeseries_usb_transport_start(
    const esp_timeseries_usb_transport_config_t *config);

/**
 * @brief Stop the command task and uninstall its USB driver.
 *
 * Called only externally, normally during teardown. It is idempotent when this
 * component owns neither resource. Do not call it concurrently with an active
 * extension callback. When disabled in Kconfig it is a no-op.
 */
void esp_timeseries_usb_transport_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* ESP_TIMESERIES_USB_TRANSPORT_H_ */
