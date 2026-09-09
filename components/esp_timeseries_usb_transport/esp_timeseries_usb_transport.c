#include "esp_timeseries_usb_transport.h"

#include "sdkconfig.h"

#if CONFIG_ESP_TIMESERIES_USB_TRANSPORT_ENABLE

#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG ||                                      \
    CONFIG_ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG
#error "Time-series USB transport requires UART-only console output"
#endif

#include "driver/usb_serial_jtag.h"
#include "esp_angle_lut.h"
#include "esp_crc.h"
#include "esp_log.h"
#include "esp_timeseries_recorder.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <ctype.h>
#include <inttypes.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#define TRANSPORT_CORE_ID 0
#define TRANSPORT_TASK_PRIORITY 3
#define TRANSPORT_TASK_STACK_SIZE 4096U
#define COMMAND_LINE_BYTES 64U
#define RESPONSE_LINE_BYTES 256U
#define USB_WRITE_CHUNK_BYTES 4096U
#define USB_READ_WAIT_MS 100U
#define PROTOCOL_VERSION 1U

_Static_assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__,
               "Recorder payload protocol requires a little-endian target");

static const char *TAG = "TS_USB";
static TaskHandle_t transport_task_handle;
static esp_timeseries_usb_transport_config_t transport_config;
/* CAL WRITE/READ are serialized by transport_task; avoid large stack arrays. */
static int16_t calibration_transfer_buffer[ESP_ANGLE_LUT_BIN_COUNT];

static TickType_t write_timeout_ticks(void) {
  return pdMS_TO_TICKS(CONFIG_ESP_TIMESERIES_USB_WRITE_TIMEOUT_MS);
}

static esp_err_t usb_send_all(const void *data, size_t length) {
  const uint8_t *cursor = (const uint8_t *)data;
  while (length > 0U) {
    size_t chunk =
        length > USB_WRITE_CHUNK_BYTES ? USB_WRITE_CHUNK_BYTES : length;
    int written =
        usb_serial_jtag_write_bytes(cursor, chunk, write_timeout_ticks());
    if (written <= 0) {
      return ESP_ERR_TIMEOUT;
    }
    cursor += (size_t)written;
    length -= (size_t)written;
  }
  return ESP_OK;
}

static esp_err_t usb_read_all(void *data, size_t length) {
  uint8_t *cursor = (uint8_t *)data;
  while (length > 0U) {
    int received = usb_serial_jtag_read_bytes(
        cursor, length,
        pdMS_TO_TICKS(CONFIG_ESP_TIMESERIES_USB_WRITE_TIMEOUT_MS));
    if (received <= 0) {
      return ESP_ERR_TIMEOUT;
    }
    cursor += (size_t)received;
    length -= (size_t)received;
  }
  return ESP_OK;
}

static esp_err_t usb_sendf(const char *format, ...) {
  char line[RESPONSE_LINE_BYTES];
  va_list arguments;
  va_start(arguments, format);
  int length = vsnprintf(line, sizeof(line), format, arguments);
  va_end(arguments);
  if (length < 0 || (size_t)length >= sizeof(line)) {
    return ESP_ERR_INVALID_SIZE;
  }
  return usb_send_all(line, (size_t)length);
}

static void send_error(const char *command, esp_err_t error,
                       const char *message) {
  usb_sendf("ERR command=%s code=%s message=%s\n", command,
            esp_err_to_name(error), message);
}

static void send_status(const char *command) {
  esp_timeseries_status_t status;
  esp_err_t error = esp_timeseries_get_status(&status);
  if (error != ESP_OK) {
    send_error(command, error, "recorder_unavailable");
    return;
  }
  usb_sendf("OK command=%s protocol=%u state=%s capture_id=%" PRIu32
            " producer_rate_hz=%" PRIu32 " sample_rate_hz=%" PRIu32
            " channels=%zu samples=%zu capacity=%zu buffer_bytes=%zu\n",
            command, PROTOCOL_VERSION, esp_timeseries_state_name(status.state),
            status.capture_id, status.producer_rate_hz, status.sample_rate_hz,
            status.channel_count, status.sample_count, status.sample_capacity,
            status.buffer_bytes);
}

static esp_err_t send_dump_header(const esp_timeseries_status_t *status,
                                  const esp_timeseries_capture_t *capture,
                                  uint32_t payload_crc32) {
  esp_err_t error = usb_sendf("TSRECORDER/%u\n", PROTOCOL_VERSION);
  if (error != ESP_OK) {
    return error;
  }
#define SEND_HEADER(...)                                                       \
  do {                                                                         \
    error = usb_sendf(__VA_ARGS__);                                            \
    if (error != ESP_OK) {                                                     \
      return error;                                                            \
    }                                                                          \
  } while (0)
  SEND_HEADER("capture_id=%" PRIu32 "\n", capture->capture_id);
  SEND_HEADER("producer_rate_hz=%" PRIu32 "\n", status->producer_rate_hz);
  SEND_HEADER("sample_rate_hz=%" PRIu32 "\n", capture->sample_rate_hz);
  SEND_HEADER("sample_count=%zu\n", capture->sample_count);
  SEND_HEADER("sample_capacity=%zu\n", capture->sample_capacity);
  SEND_HEADER("channel_count=%zu\n", capture->channel_count);
  SEND_HEADER("start_time_us=%" PRId64 "\n", capture->start_time_us);
  SEND_HEADER("encoding=int16\n");
  SEND_HEADER("byte_order=little-endian\n");
  SEND_HEADER("layout=sample-interleaved\n");
  SEND_HEADER("invalid_i16=%d\n", ESP_TIMESERIES_INVALID_I16);
  for (size_t channel = 0; channel < capture->channel_count; channel++) {
    const esp_timeseries_channel_t *descriptor = &capture->channels[channel];
    if (strchr(descriptor->name, '\n') != NULL ||
        strchr(descriptor->name, '\r') != NULL ||
        strchr(descriptor->unit, '\n') != NULL ||
        strchr(descriptor->unit, '\r') != NULL) {
      return ESP_ERR_INVALID_ARG;
    }
    SEND_HEADER("channel.%zu.name=%s\n", channel, descriptor->name);
    SEND_HEADER("channel.%zu.unit=%s\n", channel, descriptor->unit);
    SEND_HEADER("channel.%zu.scale=%.9g\n", channel, (double)descriptor->scale);
    SEND_HEADER("channel.%zu.offset=%.9g\n", channel,
                (double)descriptor->offset);
    SEND_HEADER("channel.%zu.saturation_count=%" PRIu32 "\n", channel,
                capture->saturation_counts[channel]);
    SEND_HEADER("channel.%zu.invalid_count=%" PRIu32 "\n", channel,
                capture->invalid_counts[channel]);
  }
  SEND_HEADER("payload_bytes=%zu\n", capture->payload_bytes);
  SEND_HEADER("payload_crc32=%08" PRIX32 "\n", payload_crc32);
  SEND_HEADER("END-HEADER\n");
#undef SEND_HEADER
  return ESP_OK;
}

static void send_dump(void) {
  esp_timeseries_capture_t capture;
  esp_timeseries_status_t status;
  esp_err_t error = esp_timeseries_get_capture(&capture);
  if (error != ESP_OK) {
    send_error("DUMP", error, "capture_not_full");
    return;
  }
  error = esp_timeseries_get_status(&status);
  if (error != ESP_OK || status.state != ESP_TIMESERIES_STATE_FULL) {
    send_error("DUMP", ESP_ERR_INVALID_STATE, "capture_not_stable");
    return;
  }

  uint32_t crc32 = esp_crc32_le(0U, (const uint8_t *)capture.samples,
                                (uint32_t)capture.payload_bytes);
  error = send_dump_header(&status, &capture, crc32);
  if (error == ESP_OK) {
    error = usb_send_all(capture.samples, capture.payload_bytes);
  }
  if (error == ESP_OK) {
    error = usb_serial_jtag_wait_tx_done(write_timeout_ticks());
  }
  if (error != ESP_OK) {
    ESP_LOGW(TAG, "DUMP %" PRIu32 " interrupted: %s", capture.capture_id,
             esp_err_to_name(error));
  }
}

static bool parse_arm_rate(const char *line, uint32_t *rate_hz) {
  if (strncasecmp(line, "ARM", 3U) != 0 || !isspace((unsigned char)line[3])) {
    return false;
  }
  const char *value = line + 3U;
  while (isspace((unsigned char)*value)) {
    value++;
  }
  char *end = NULL;
  unsigned long parsed = strtoul(value, &end, 10);
  if (end == value) {
    return false;
  }
  while (isspace((unsigned char)*end)) {
    end++;
  }
  if (*end != '\0' || parsed > UINT32_MAX) {
    return false;
  }
  *rate_hz = (uint32_t)parsed;
  return true;
}

static bool parse_named_rate(const char *line, const char *command,
                             uint32_t *rate_hz) {
  size_t command_length = strlen(command);
  if (strncasecmp(line, command, command_length) != 0 ||
      !isspace((unsigned char)line[command_length])) {
    return false;
  }
  const char *value = line + command_length;
  while (isspace((unsigned char)*value)) {
    value++;
  }
  char *end = NULL;
  unsigned long parsed = strtoul(value, &end, 10);
  if (end == value) {
    return false;
  }
  while (isspace((unsigned char)*end)) {
    end++;
  }
  if (*end != '\0' || parsed > UINT32_MAX) {
    return false;
  }
  *rate_hz = (uint32_t)parsed;
  return true;
}

static esp_err_t arm_capture(uint32_t rate_hz, bool calibration_mode) {
  if (transport_config.arm_handler != NULL) {
    return transport_config.arm_handler(rate_hz, calibration_mode,
                                        transport_config.arm_handler_context);
  }
  return calibration_mode ? ESP_ERR_NOT_SUPPORTED : esp_timeseries_arm(rate_hz);
}

static const char *arm_error_message(esp_err_t error) {
  if (error == ESP_ERR_INVALID_ARG) {
    return "invalid_sample_rate";
  }
  if (error == ESP_ERR_NOT_SUPPORTED) {
    return "calibration_profile_disabled";
  }
  return "recorder_not_empty";
}

static void send_calibration_status(const char *command) {
  esp_angle_lut_status_t status;
  esp_angle_lut_get_status(&status);
  usb_sendf("OK command=%s protocol=%u loaded=%u enabled=%u format=%u "
            "bins=%u full_scale_counts=%" PRIu32
            " max_abs_correction_counts=%" PRIu32 " generation=%" PRIu32
            " crc32=%08" PRIX32 "\n",
            command, PROTOCOL_VERSION, status.loaded, status.enabled,
            status.format_version, status.bin_count, status.full_scale_counts,
            status.max_abs_correction_counts, status.generation,
            status.payload_crc32);
}

static bool parse_cal_write(const char *line, size_t *bin_count,
                            uint32_t *full_scale_counts,
                            uint32_t *payload_crc32) {
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

static void receive_calibration(size_t bin_count, uint32_t full_scale_counts,
                                uint32_t payload_crc32) {
  if (bin_count != ESP_ANGLE_LUT_BIN_COUNT) {
    send_error("CAL_WRITE", ESP_ERR_INVALID_SIZE, "unexpected_bin_count");
    return;
  }
  if (full_scale_counts != ESP_ANGLE_LUT_FULL_SCALE_COUNTS) {
    send_error("CAL_WRITE", ESP_ERR_INVALID_SIZE,
               "unexpected_full_scale_counts");
    return;
  }
  esp_timeseries_state_t recorder_state = esp_timeseries_get_state();
  if (recorder_state == ESP_TIMESERIES_STATE_ARMED ||
      recorder_state == ESP_TIMESERIES_STATE_CAPTURING) {
    send_error("CAL_WRITE", ESP_ERR_INVALID_STATE, "capture_in_progress");
    return;
  }

  esp_err_t error =
      usb_sendf("OK command=CAL_WRITE state=READY bytes=%zu bins=%zu\n",
                sizeof(calibration_transfer_buffer), bin_count);
  if (error == ESP_OK) {
    error = usb_read_all(calibration_transfer_buffer,
                         sizeof(calibration_transfer_buffer));
  }
  esp_angle_lut_install_failure_t failure = ESP_ANGLE_LUT_INSTALL_FAILURE_NONE;
  if (error == ESP_OK) {
    error = esp_angle_lut_install_detailed(calibration_transfer_buffer,
                                           bin_count, full_scale_counts,
                                           payload_crc32, &failure);
  }
  if (error != ESP_OK) {
    const char *reason = error == ESP_ERR_TIMEOUT
                             ? "payload_timeout"
                             : esp_angle_lut_install_failure_name(failure);
    ESP_LOGE(
        TAG, "CAL WRITE failed: reason=%s error=%s stack_free_min=%u B", reason,
        esp_err_to_name(error),
        (unsigned)(uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t)));
    send_error("CAL_WRITE", error, reason);
    return;
  }
  ESP_LOGI(TAG, "CAL WRITE installed; stack_free_min=%u B",
           (unsigned)(uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t)));
  send_calibration_status("CAL_WRITE");
}

static void send_calibration(void) {
  esp_angle_lut_status_t status;
  esp_err_t error = esp_angle_lut_read(calibration_transfer_buffer,
                                       ESP_ANGLE_LUT_BIN_COUNT, &status);
  if (error != ESP_OK) {
    send_error("CAL_READ", error, "calibration_not_loaded");
    return;
  }
  error = usb_sendf("ANGLELUT/%u\n", ESP_ANGLE_LUT_FORMAT_VERSION);
  if (error == ESP_OK) {
    error = usb_sendf("bins=%u\n", status.bin_count);
  }
  if (error == ESP_OK) {
    error =
        usb_sendf("full_scale_counts=%" PRIu32 "\n", status.full_scale_counts);
  }
  if (error == ESP_OK) {
    error = usb_sendf("generation=%" PRIu32 "\n", status.generation);
  }
  if (error == ESP_OK) {
    error = usb_sendf("enabled=%u\n", status.enabled);
  }
  if (error == ESP_OK) {
    error = usb_sendf("encoding=int16\nbyte_order=little-endian\n");
  }
  if (error == ESP_OK) {
    error = usb_sendf(
        "payload_bytes=%zu\npayload_crc32=%08" PRIX32 "\nEND-HEADER\n",
        sizeof(calibration_transfer_buffer), status.payload_crc32);
  }
  if (error == ESP_OK) {
    error = usb_send_all(calibration_transfer_buffer,
                         sizeof(calibration_transfer_buffer));
  }
  if (error != ESP_OK) {
    ESP_LOGW(TAG, "CAL READ interrupted: %s", esp_err_to_name(error));
  }
}

static bool process_calibration_command(const char *line) {
  if (strcasecmp(line, "CAL STATUS") == 0) {
    send_calibration_status("CAL_STATUS");
  } else if (strcasecmp(line, "CAL READ") == 0) {
    send_calibration();
  } else if (strcasecmp(line, "CAL ENABLE") == 0 ||
             strcasecmp(line, "CAL DISABLE") == 0) {
    bool enabled = strcasecmp(line, "CAL ENABLE") == 0;
    esp_err_t error = esp_angle_lut_set_enabled(enabled);
    if (error == ESP_OK) {
      send_calibration_status(enabled ? "CAL_ENABLE" : "CAL_DISABLE");
    } else {
      send_error(enabled ? "CAL_ENABLE" : "CAL_DISABLE", error,
                 "calibration_not_loaded");
    }
  } else if (strcasecmp(line, "CAL CLEAR") == 0) {
    esp_err_t error = esp_angle_lut_clear();
    if (error == ESP_OK) {
      send_calibration_status("CAL_CLEAR");
    } else {
      send_error("CAL_CLEAR", error, "nvs_error");
    }
  } else {
    uint32_t rate_hz = 0U;
    if (parse_named_rate(line, "CAL START", &rate_hz)) {
      esp_err_t error = arm_capture(rate_hz, true);
      if (error == ESP_OK) {
        send_status("CAL_START");
      } else {
        send_error("CAL_START", error, arm_error_message(error));
      }
      return true;
    }
    size_t bin_count = 0U;
    uint32_t full_scale_counts = 0U;
    uint32_t crc32 = 0U;
    if (!parse_cal_write(line, &bin_count, &full_scale_counts, &crc32)) {
      return false;
    }
    receive_calibration(bin_count, full_scale_counts, crc32);
  }
  return true;
}

static void process_command(const char *line) {
  if (strcasecmp(line, "PING") == 0) {
    usb_sendf("OK command=PING protocol=%u\n", PROTOCOL_VERSION);
  } else if (strcasecmp(line, "INFO") == 0 || strcasecmp(line, "STATUS") == 0) {
    send_status(strcasecmp(line, "INFO") == 0 ? "INFO" : "STATUS");
  } else if (strcasecmp(line, "DUMP") == 0) {
    send_dump();
  } else if (strcasecmp(line, "CLEAR") == 0) {
    esp_err_t error = esp_timeseries_clear();
    if (error == ESP_OK) {
      send_status("CLEAR");
    } else {
      send_error("CLEAR", error, "capture_not_full");
    }
  } else if (strcasecmp(line, "HELP") == 0) {
    usb_sendf("OK command=HELP commands=PING,INFO,STATUS,ARM_<hz>,DUMP,CLEAR "
              "CAL_START_<hz>,CAL_STATUS,CAL_WRITE,CAL_READ,CAL_ENABLE,"
              "CAL_DISABLE,CAL_CLEAR "
              "rate_rule=exact_divisor_of_producer_rate\n");
  } else if (strncasecmp(line, "CAL ", 4U) == 0 &&
             process_calibration_command(line)) {
    return;
  } else {
    uint32_t rate_hz = 0U;
    if (parse_arm_rate(line, &rate_hz)) {
      esp_err_t error = arm_capture(rate_hz, false);
      if (error == ESP_OK) {
        send_status("ARM");
      } else {
        send_error("ARM", error, arm_error_message(error));
      }
    } else {
      send_error("UNKNOWN", ESP_ERR_INVALID_ARG, "use_HELP");
    }
  }
}

static void transport_task(void *argument) {
  (void)argument;
  char line[COMMAND_LINE_BYTES];
  size_t line_length = 0U;
  bool line_overflow = false;
  uint8_t input[64];

  ESP_LOGI(TAG, "Recorder protocol ready on native USB Serial/JTAG");
  while (true) {
    int received = usb_serial_jtag_read_bytes(input, sizeof(input),
                                              pdMS_TO_TICKS(USB_READ_WAIT_MS));
    for (int index = 0; index < received; index++) {
      char character = (char)input[index];
      if (character == '\r') {
        continue;
      }
      if (character == '\n') {
        if (line_overflow) {
          send_error("UNKNOWN", ESP_ERR_INVALID_SIZE, "command_too_long");
        } else if (line_length > 0U) {
          line[line_length] = '\0';
          process_command(line);
        }
        line_length = 0U;
        line_overflow = false;
      } else if (!line_overflow) {
        if (line_length + 1U < sizeof(line)) {
          line[line_length++] = character;
        } else {
          line_overflow = true;
        }
      }
    }
  }
}

esp_err_t esp_timeseries_usb_transport_start(
    const esp_timeseries_usb_transport_config_t *config) {
  if (transport_task_handle != NULL || usb_serial_jtag_is_driver_installed()) {
    return ESP_ERR_INVALID_STATE;
  }
  transport_config =
      config != NULL ? *config : (esp_timeseries_usb_transport_config_t){0};
  usb_serial_jtag_driver_config_t driver_config = {
      .tx_buffer_size = CONFIG_ESP_TIMESERIES_USB_TX_BUFFER_BYTES,
      .rx_buffer_size = CONFIG_ESP_TIMESERIES_USB_RX_BUFFER_BYTES,
  };
  esp_err_t error = usb_serial_jtag_driver_install(&driver_config);
  if (error != ESP_OK) {
    return error;
  }
  BaseType_t created = xTaskCreatePinnedToCore(
      transport_task, "timeseries_usb", TRANSPORT_TASK_STACK_SIZE, NULL,
      TRANSPORT_TASK_PRIORITY, &transport_task_handle, TRANSPORT_CORE_ID);
  if (created != pdPASS) {
    transport_task_handle = NULL;
    usb_serial_jtag_driver_uninstall();
    return ESP_ERR_NO_MEM;
  }
  return ESP_OK;
}

void esp_timeseries_usb_transport_stop(void) {
  if (transport_task_handle != NULL) {
    vTaskDelete(transport_task_handle);
    transport_task_handle = NULL;
  }
  if (usb_serial_jtag_is_driver_installed()) {
    usb_serial_jtag_driver_uninstall();
  }
}

#else

esp_err_t esp_timeseries_usb_transport_start(
    const esp_timeseries_usb_transport_config_t *config) {
  (void)config;
  return ESP_OK;
}

void esp_timeseries_usb_transport_stop(void) {}

#endif
