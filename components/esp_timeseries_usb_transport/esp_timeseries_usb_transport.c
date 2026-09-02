#include "esp_timeseries_usb_transport.h"

#include "sdkconfig.h"

#if CONFIG_ESP_TIMESERIES_USB_TRANSPORT_ENABLE

#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG ||                                      \
    CONFIG_ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG
#error "Time-series USB transport requires UART-only console output"
#endif

#include "driver/usb_serial_jtag.h"
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
              "rate_rule=exact_divisor_of_producer_rate\n");
  } else {
    uint32_t rate_hz = 0U;
    if (parse_arm_rate(line, &rate_hz)) {
      esp_err_t error = esp_timeseries_arm(rate_hz);
      if (error == ESP_OK) {
        send_status("ARM");
      } else {
        send_error("ARM", error,
                   error == ESP_ERR_INVALID_ARG ? "invalid_sample_rate"
                                                : "recorder_not_empty");
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

esp_err_t esp_timeseries_usb_transport_start(void) {
  if (transport_task_handle != NULL || usb_serial_jtag_is_driver_installed()) {
    return ESP_ERR_INVALID_STATE;
  }
  usb_serial_jtag_driver_config_t config = {
      .tx_buffer_size = CONFIG_ESP_TIMESERIES_USB_TX_BUFFER_BYTES,
      .rx_buffer_size = CONFIG_ESP_TIMESERIES_USB_RX_BUFFER_BYTES,
  };
  esp_err_t error = usb_serial_jtag_driver_install(&config);
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

esp_err_t esp_timeseries_usb_transport_start(void) { return ESP_OK; }

void esp_timeseries_usb_transport_stop(void) {}

#endif
