#include "angle_lut_usb_commands.h"

#include "esp_angle_lut.h"
#include "esp_log.h"
#include "esp_timeseries_recorder.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>

#define ANGLE_LUT_USB_PROTOCOL_VERSION 1U

static const char *TAG = "ANGLE_LUT_USB";

/* The USB command task serializes CAL WRITE and CAL READ operations. Static
 * storage avoids placing the complete LUT on that task's stack. */
static int16_t calibration_transfer_buffer[ESP_ANGLE_LUT_BIN_COUNT];

/**
 * @brief Send current LUT metadata using the protocol's key/value format.
 *
 * Called internally by angle_lut_usb_command_handler() and
 * receive_calibration(). It queries esp_angle_lut and synchronously emits one
 * textual line; no state or pointer is retained.
 *
 * @param io Valid USB command response interface.
 * @param command Response command token such as CAL_STATUS or CAL_WRITE.
 */
static void send_calibration_status(const esp_timeseries_usb_command_io_t *io,
                                    const char *command) {
  /* Snapshot block: obtain a coherent by-value description of the LUT. */
  esp_angle_lut_status_t status;
  esp_angle_lut_get_status(&status);
  /* Serialization block: preserve fixed field names consumed by Octave. */
  io->sendf("OK command=%s protocol=%u loaded=%u enabled=%u format=%u "
            "bins=%u full_scale_counts=%" PRIu32
            " max_abs_correction_counts=%" PRIu32 " generation=%" PRIu32
            " crc32=%08" PRIX32 "\n",
            command, ANGLE_LUT_USB_PROTOCOL_VERSION, status.loaded,
            status.enabled, status.format_version, status.bin_count,
            status.full_scale_counts, status.max_abs_correction_counts,
            status.generation, status.payload_crc32);
}

/**
 * @brief Parse an exact command prefix followed by one unsigned rate.
 *
 * Called internally only by angle_lut_usb_command_handler() for CAL START.
 * The output is written only after syntax and uint32 range validation.
 *
 * @param line Complete NUL-terminated command line.
 * @param command Expected command prefix without the numeric argument.
 * @param rate_hz Destination for the positive-domain integer syntax; semantic
 * rate validation is delegated to the recorder arm callback.
 * @return true when exactly one uint32-compatible number follows the prefix.
 */
static bool parse_named_rate(const char *line, const char *command,
                             uint32_t *rate_hz) {
  /* Prefix block: require a separating space to avoid partial-name matches. */
  size_t command_length = strlen(command);
  if (strncasecmp(line, command, command_length) != 0 ||
      line[command_length] != ' ') {
    return false;
  }

  /* Numeric block: a second conversion catches any trailing non-space data. */
  unsigned long parsed_rate = 0UL;
  char extra = '\0';
  int fields = sscanf(line + command_length, " %lu %c", &parsed_rate, &extra);
  if (fields != 1 || parsed_rate > UINT32_MAX) {
    return false;
  }
  *rate_hz = (uint32_t)parsed_rate;
  return true;
}

/**
 * @brief Parse CAL WRITE table size, angular resolution, and payload CRC32.
 *
 * Called internally only by angle_lut_usb_command_handler(). It validates host
 * integer-width conversions but leaves component-specific size checks to
 * receive_calibration(). Output fields are written only on success.
 *
 * @return true when exactly three representable metadata values are present.
 */
static bool parse_cal_write(const char *line, size_t *bin_count,
                            uint32_t *full_scale_counts,
                            uint32_t *payload_crc32) {
  /* Parse block: hexadecimal %lx accepts the eight-digit CRC emitted by Octave;
   * the extra character conversion rejects trailing protocol fields. */
  unsigned long parsed_bins = 0UL;
  unsigned long parsed_full_scale = 0UL;
  unsigned long parsed_crc = 0UL;
  char extra = '\0';
  int fields = sscanf(line, "CAL WRITE %lu %lu %lx %c", &parsed_bins,
                      &parsed_full_scale, &parsed_crc, &extra);
  if (fields != 3 || parsed_bins > SIZE_MAX || parsed_full_scale > UINT32_MAX ||
      parsed_crc > UINT32_MAX) {
    return false;
  }
  *bin_count = (size_t)parsed_bins;
  *full_scale_counts = (uint32_t)parsed_full_scale;
  *payload_crc32 = (uint32_t)parsed_crc;
  return true;
}

/**
 * @brief Receive, validate, and persist one binary correction table.
 *
 * Called internally only by angle_lut_usb_command_handler() after textual
 * metadata parsing. It runs in the serialized USB task, uses the module-static
 * transfer buffer, and may block until the binary payload arrives or times out.
 * It calls esp_angle_lut_install_detailed(), which verifies limits and CRC
 * before replacing the installed LUT.
 */
static void receive_calibration(const esp_timeseries_usb_command_io_t *io,
                                size_t bin_count, uint32_t full_scale_counts,
                                uint32_t payload_crc32) {
  /* Compatibility block: reject a table whose dimensions differ from this
   * firmware before inviting the host to transmit any binary bytes. */
  if (bin_count != ESP_ANGLE_LUT_BIN_COUNT) {
    io->send_error("CAL_WRITE", ESP_ERR_INVALID_SIZE, "unexpected_bin_count");
    return;
  }
  if (full_scale_counts != ESP_ANGLE_LUT_FULL_SCALE_COUNTS) {
    io->send_error("CAL_WRITE", ESP_ERR_INVALID_SIZE,
                   "unexpected_full_scale_counts");
    return;
  }

  /* Exclusion block: do not perform flash installation during acquisition. */
  esp_timeseries_state_t recorder_state = esp_timeseries_get_state();
  if (recorder_state == ESP_TIMESERIES_STATE_ARMED ||
      recorder_state == ESP_TIMESERIES_STATE_CAPTURING) {
    io->send_error("CAL_WRITE", ESP_ERR_INVALID_STATE, "capture_in_progress");
    return;
  }

  /* Transfer block: READY is the host's permission to send the exact payload.
   */
  esp_err_t error =
      io->sendf("OK command=CAL_WRITE state=READY bytes=%zu bins=%zu\n",
                sizeof(calibration_transfer_buffer), bin_count);
  if (error == ESP_OK) {
    error = io->read_all(calibration_transfer_buffer,
                         sizeof(calibration_transfer_buffer));
  }

  /* Installation block: preserve the component's detailed validation reason. */
  esp_angle_lut_install_failure_t failure = ESP_ANGLE_LUT_INSTALL_FAILURE_NONE;
  if (error == ESP_OK) {
    error = esp_angle_lut_install_detailed(calibration_transfer_buffer,
                                           bin_count, full_scale_counts,
                                           payload_crc32, &failure);
  }
  /* Result block: translate transport timeouts and validation failures into a
   * stable protocol error, while logging stack headroom for diagnosis. */
  if (error != ESP_OK) {
    const char *reason = error == ESP_ERR_TIMEOUT
                             ? "payload_timeout"
                             : esp_angle_lut_install_failure_name(failure);
    ESP_LOGE(
        TAG, "CAL WRITE failed: reason=%s error=%s stack_free_min=%u B", reason,
        esp_err_to_name(error),
        (unsigned)(uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t)));
    io->send_error("CAL_WRITE", error, reason);
    return;
  }

  ESP_LOGI(TAG, "CAL WRITE installed; stack_free_min=%u B",
           (unsigned)(uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t)));
  send_calibration_status(io, "CAL_WRITE");
}

/**
 * @brief Send the installed LUT with a textual header and binary payload.
 *
 * Called internally only by angle_lut_usb_command_handler() for CAL READ. The
 * header declares format, dimensions, little-endian int16 encoding, length,
 * and CRC before write_all() sends the static buffer.
 */
static void send_calibration(const esp_timeseries_usb_command_io_t *io) {
  /* Read block: copy the installed table and its coherent metadata. */
  esp_angle_lut_status_t status;
  esp_err_t error = esp_angle_lut_read(calibration_transfer_buffer,
                                       ESP_ANGLE_LUT_BIN_COUNT, &status);
  if (error != ESP_OK) {
    io->send_error("CAL_READ", error, "calibration_not_loaded");
    return;
  }

  /* Header block: stop at the first transport error to preserve framing. */
  error = io->sendf("ANGLELUT/%u\n", ESP_ANGLE_LUT_FORMAT_VERSION);
  if (error == ESP_OK) {
    error = io->sendf("bins=%u\n", status.bin_count);
  }
  if (error == ESP_OK) {
    error =
        io->sendf("full_scale_counts=%" PRIu32 "\n", status.full_scale_counts);
  }
  if (error == ESP_OK) {
    error = io->sendf("generation=%" PRIu32 "\n", status.generation);
  }
  if (error == ESP_OK) {
    error = io->sendf("enabled=%u\n", status.enabled);
  }
  if (error == ESP_OK) {
    error = io->sendf("encoding=int16\nbyte_order=little-endian\n");
  }
  if (error == ESP_OK) {
    error = io->sendf(
        "payload_bytes=%zu\npayload_crc32=%08" PRIX32 "\nEND-HEADER\n",
        sizeof(calibration_transfer_buffer), status.payload_crc32);
  }
  /* Payload block: write the exact byte count declared in the header. */
  if (error == ESP_OK) {
    error = io->write_all(calibration_transfer_buffer,
                          sizeof(calibration_transfer_buffer));
  }
  if (error != ESP_OK) {
    ESP_LOGW(TAG, "CAL READ interrupted: %s", esp_err_to_name(error));
  }
}

/**
 * @brief Dispatch one CAL extension command from the generic USB transport.
 *
 * Called by application_usb_command_handler(). See the public header for
 * ownership, execution-context, and return-contract details.
 */
bool angle_lut_usb_command_handler(const char *command,
                                   const esp_timeseries_usb_command_io_t *io,
                                   void *context) {
  /* Namespace block: decline non-CAL commands without producing output. */
  if (command == NULL || io == NULL || strncasecmp(command, "CAL ", 4U) != 0) {
    return false;
  }

  /* Direct-command block: serve status, transfer, enable, and persistence
   * commands before attempting commands that carry positional arguments. */
  const angle_lut_usb_command_context_t *calibration_context =
      (const angle_lut_usb_command_context_t *)context;
  if (strcasecmp(command, "CAL STATUS") == 0) {
    send_calibration_status(io, "CAL_STATUS");
  } else if (strcasecmp(command, "CAL READ") == 0) {
    send_calibration(io);
  } else if (strcasecmp(command, "CAL ENABLE") == 0 ||
             strcasecmp(command, "CAL DISABLE") == 0) {
    bool enabled = strcasecmp(command, "CAL ENABLE") == 0;
    esp_err_t error = esp_angle_lut_set_enabled(enabled);
    if (error == ESP_OK) {
      send_calibration_status(io, enabled ? "CAL_ENABLE" : "CAL_DISABLE");
    } else {
      io->send_error(enabled ? "CAL_ENABLE" : "CAL_DISABLE", error,
                     "calibration_not_loaded");
    }
  } else if (strcasecmp(command, "CAL CLEAR") == 0) {
    esp_err_t error = esp_angle_lut_clear();
    if (error == ESP_OK) {
      send_calibration_status(io, "CAL_CLEAR");
    } else {
      io->send_error("CAL_CLEAR", error, "nvs_error");
    }
  } else {
    /* CAL START block: let the application atomically coordinate recorder ARM
     * with selection of the open-loop calibration profile. */
    uint32_t rate_hz = 0U;
    if (parse_named_rate(command, "CAL START", &rate_hz)) {
      if (calibration_context == NULL ||
          calibration_context->calibration_arm_handler == NULL) {
        io->send_error("CAL_START", ESP_ERR_NOT_SUPPORTED,
                       "calibration_profile_disabled");
        return true;
      }
      esp_err_t error = calibration_context->calibration_arm_handler(
          rate_hz, calibration_context->calibration_arm_context);
      if (error == ESP_OK) {
        io->send_recorder_status("CAL_START");
      } else {
        const char *reason = error == ESP_ERR_INVALID_ARG
                                 ? "invalid_sample_rate"
                                 : "recorder_not_empty";
        io->send_error("CAL_START", error, reason);
      }
      return true;
    }

    /* CAL WRITE block: parse metadata, then run the binary receive handshake.
     */
    size_t bin_count = 0U;
    uint32_t full_scale_counts = 0U;
    uint32_t payload_crc32 = 0U;
    if (parse_cal_write(command, &bin_count, &full_scale_counts,
                        &payload_crc32)) {
      receive_calibration(io, bin_count, full_scale_counts, payload_crc32);
    } else {
      io->send_error("CAL", ESP_ERR_INVALID_ARG, "invalid_calibration_command");
    }
  }
  return true;
}
