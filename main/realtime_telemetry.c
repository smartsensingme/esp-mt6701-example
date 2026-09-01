/*
 * Non-real-time presentation layer.
 *
 * Core 1 only calls realtime_telemetry_publish() to copy a snapshot. All
 * formatting, floating-point rate calculation, window selection, and ESP_LOGI
 * output happen in the low-priority Core 0 task defined here.
 */
#include "realtime_telemetry.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <inttypes.h>
#include <stdbool.h>

#define TELEMETRY_CORE_ID 0
#define TELEMETRY_TASK_STACK_SIZE 4096U
#define TELEMETRY_TASK_PRIORITY 1
#define MICROSECONDS_TO_SECONDS (1.0f / 1000000.0f)

static const char *TAG = "REALTIME_LOOP";

/* Capacity one is intentional: only the newest diagnostic state is relevant. */
static QueueHandle_t telemetry_queue;
static TaskHandle_t telemetry_task_handle;

static void telemetry_logger_task(void *argument) {
  (void)argument;
  realtime_telemetry_snapshot_t telemetry;
  bool discard_contaminated_window = false;

  ESP_LOGI(TAG, "Telemetry logger running on Core %d", xPortGetCoreID());
  while (true) {
    if (xQueueReceive(telemetry_queue, &telemetry, portMAX_DELAY) != pdTRUE) {
      continue;
    }

    /*
     * Serial output can disturb shared SoC resources during the following
     * five-second timing window. Print one silent window, discard the next one,
     * then print again. Snapshots are still produced every five seconds.
     */
    if (discard_contaminated_window) {
      discard_contaminated_window = false;
      continue;
    }
    discard_contaminated_window = true;

    /* Rates use the measured window duration, not an assumed exact five
     * seconds. */
    float window_seconds =
        (float)telemetry.window_duration_us * MICROSECONDS_TO_SECONDS;
    float estimator_rate_hz =
        (float)telemetry.estimator_updates / window_seconds;
    float control_rate_hz = (float)telemetry.control_updates / window_seconds;

    ESP_LOGI(TAG,
             "silent window: angle=%.3f/%.3f deg speed=%.3f RPM "
             "accel=%.3f RPM/s turns=%" PRId32,
             telemetry.measured_angle_deg, telemetry.estimated_angle_deg,
             telemetry.estimated_speed_rpm,
             telemetry.estimated_acceleration_rpm_s, telemetry.total_turns);
    ESP_LOGI(TAG,
             "control: reference=%.1f RPM error=%.1f RPM "
             "pid=%.2f/%.2f/%.2f%% output=%.1f%%",
             telemetry.speed_reference_rpm, telemetry.speed_error_rpm,
             telemetry.pid_proportional_term, telemetry.pid_integral_term,
             telemetry.pid_derivative_term, telemetry.motor_output_percent);
    ESP_LOGI(TAG,
             "window: rate=%.1f/%.1f Hz missed=%" PRIu32 " errors=%" PRIu32
             " overruns=%" PRIu32,
             estimator_rate_hz, control_rate_hz, telemetry.missed_timer_events,
             telemetry.sensor_errors, telemetry.deadline_overruns);
    ESP_LOGI(TAG,
             "window max: wake=%" PRIu32 " i2c=%" PRIu32 " kalman=%" PRIu32
             " control=%" PRIu32 " processing=%" PRIu32 " cycle=%" PRIu32
             " us lifetime_processing=%" PRIu32
             " us previous_telemetry=%" PRIu32 " us",
             telemetry.max_wake_latency_us, telemetry.max_i2c_time_us,
             telemetry.max_kalman_time_us, telemetry.max_control_time_us,
             telemetry.max_processing_time_us, telemetry.max_cycle_time_us,
             telemetry.lifetime_max_processing_time_us,
             telemetry.previous_telemetry_time_us);
    ESP_LOGI(TAG,
             "window dt: sample=%" PRIu32 "..%" PRIu32 " us control=%" PRIu32
             "..%" PRIu32 " us totals: est=%" PRIu32 " ctl=%" PRIu32
             " missed=%" PRIu32 " errors=%" PRIu32,
             telemetry.min_sample_dt_us, telemetry.max_sample_dt_us,
             telemetry.min_control_dt_us, telemetry.max_control_dt_us,
             telemetry.total_estimator_updates, telemetry.total_control_updates,
             telemetry.total_missed_timer_events,
             telemetry.total_sensor_errors);
  }
}

esp_err_t realtime_telemetry_start(void) {
  if (telemetry_queue != NULL || telemetry_task_handle != NULL) {
    return ESP_ERR_INVALID_STATE;
  }

  telemetry_queue = xQueueCreate(1, sizeof(realtime_telemetry_snapshot_t));
  if (telemetry_queue == NULL) {
    return ESP_ERR_NO_MEM;
  }

  BaseType_t task_created = xTaskCreatePinnedToCore(
      telemetry_logger_task, "telemetry_logger", TELEMETRY_TASK_STACK_SIZE,
      NULL, TELEMETRY_TASK_PRIORITY, &telemetry_task_handle, TELEMETRY_CORE_ID);
  if (task_created != pdPASS) {
    vQueueDelete(telemetry_queue);
    telemetry_queue = NULL;
    return ESP_ERR_NO_MEM;
  }

  return ESP_OK;
}

void realtime_telemetry_stop(void) {
  if (telemetry_task_handle != NULL) {
    vTaskDelete(telemetry_task_handle);
    telemetry_task_handle = NULL;
  }
  if (telemetry_queue != NULL) {
    vQueueDelete(telemetry_queue);
    telemetry_queue = NULL;
  }
}

void realtime_telemetry_publish(const realtime_telemetry_snapshot_t *snapshot) {
  if (snapshot != NULL && telemetry_queue != NULL) {
    xQueueOverwrite(telemetry_queue, snapshot);
  }
}
