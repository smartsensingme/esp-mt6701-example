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

static void log_snapshot(const realtime_telemetry_snapshot_t *telemetry,
                         const char *window_class) {
  const esp_rt_diag_snapshot_t *diagnostics = &telemetry->diagnostics;
  float window_seconds =
      (float)diagnostics->window_duration_us * MICROSECONDS_TO_SECONDS;
  float estimator_rate_hz =
      (float)diagnostics->event_counts[REALTIME_DIAG_EVENT_ESTIMATOR_UPDATE] /
      window_seconds;
  float control_rate_hz =
      (float)diagnostics->event_counts[REALTIME_DIAG_EVENT_CONTROL_UPDATE] /
      window_seconds;

  ESP_LOGI(TAG,
           "snapshot[%s]: duration=%.3f s rate=%.1f/%.1f Hz "
           "cycles=%" PRIu32 " total_cycles=%" PRIu32 " wake_valid=%" PRIu32,
           window_class, window_seconds, estimator_rate_hz, control_rate_hz,
           diagnostics->cycles, diagnostics->total_cycles,
           diagnostics->cycles_with_valid_wake_time);
  ESP_LOGI(TAG,
           "events[%s]: missed=%" PRIu32 "/%" PRIu32 " errors=%" PRIu32
           "/%" PRIu32 " overruns=%" PRIu32 "/%" PRIu32 " (window/total)",
           window_class, diagnostics->missed_events,
           diagnostics->total_missed_events,
           diagnostics->event_counts[REALTIME_DIAG_EVENT_SENSOR_ERROR],
           diagnostics->total_event_counts[REALTIME_DIAG_EVENT_SENSOR_ERROR],
           diagnostics->deadline_overruns,
           diagnostics->total_deadline_overruns);
  ESP_LOGI(TAG,
           "timing[%s]: max wake=%" PRIu32 " i2c=%" PRIu32 " kalman=%" PRIu32
           " control=%" PRIu32 " processing=%" PRIu32 " cycle=%" PRIu32
           " us lifetime_processing=%" PRIu32 " us previous_snapshot=%" PRIu32
           " us",
           window_class, diagnostics->max_wake_latency_us,
           diagnostics->stages[REALTIME_DIAG_STAGE_I2C].max_duration_us,
           diagnostics->stages[REALTIME_DIAG_STAGE_KALMAN].max_duration_us,
           diagnostics->stages[REALTIME_DIAG_STAGE_CONTROL].max_duration_us,
           diagnostics->max_processing_time_us, diagnostics->max_cycle_time_us,
           diagnostics->lifetime_max_processing_time_us,
           diagnostics->stages[REALTIME_DIAG_STAGE_SNAPSHOT].max_duration_us);
  ESP_LOGI(TAG,
           "state[%s]: angle=%.3f/%.3f deg speed=%.3f RPM "
           "accel=%.3f RPM/s turns=%" PRId32 " reference=%.1f RPM "
           "error=%.1f RPM pid=%.2f/%.2f/%.2f%% output=%.1f%%",
           window_class, telemetry->measured_angle_deg,
           telemetry->estimated_angle_deg, telemetry->estimated_speed_rpm,
           telemetry->estimated_acceleration_rpm_s, telemetry->total_turns,
           telemetry->speed_reference_rpm, telemetry->speed_error_rpm,
           telemetry->pid_proportional_term, telemetry->pid_integral_term,
           telemetry->pid_derivative_term, telemetry->motor_output_percent);
  ESP_LOGI(TAG,
           "interval[%s]: sample=%" PRIu32 "..%" PRIu32 " us control=%" PRIu32
           "..%" PRIu32 " us",
           window_class,
           diagnostics->intervals[REALTIME_DIAG_INTERVAL_SAMPLE].min_us,
           diagnostics->intervals[REALTIME_DIAG_INTERVAL_SAMPLE].max_us,
           diagnostics->intervals[REALTIME_DIAG_INTERVAL_CONTROL].min_us,
           diagnostics->intervals[REALTIME_DIAG_INTERVAL_CONTROL].max_us);
}

static void telemetry_logger_task(void *argument) {
  (void)argument;
  realtime_telemetry_snapshot_t telemetry;
  realtime_telemetry_snapshot_t deferred_log_affected;
  /* The first window may contain startup logs, so classify it as affected. */
  bool next_window_is_log_affected = true;
  bool have_deferred_log_affected = false;

  ESP_LOGI(TAG, "Telemetry logger running on Core %d", xPortGetCoreID());
  while (true) {
    if (xQueueReceive(telemetry_queue, &telemetry, portMAX_DELAY) != pdTRUE) {
      continue;
    }

    /*
     * Serial output can disturb shared SoC resources during the measurement
     * window that follows it. Preserve that snapshot without printing, then
     * report it beside the subsequent quiet window. This keeps the act of
     * reporting confined to alternating windows without hiding its impact.
     */
    if (next_window_is_log_affected) {
      deferred_log_affected = telemetry;
      have_deferred_log_affected = true;
      next_window_is_log_affected = false;
      continue;
    }

    if (have_deferred_log_affected) {
      log_snapshot(&deferred_log_affected, "log-affected");
      have_deferred_log_affected = false;
    }
    log_snapshot(&telemetry, "quiet");
    next_window_is_log_affected = true;
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
