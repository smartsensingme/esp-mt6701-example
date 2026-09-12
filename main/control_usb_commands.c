#include "control_usb_commands.h"

#include <stdio.h>
#include <string.h>
#include <strings.h>

#define CONTROL_USB_PROTOCOL_VERSION 1U
#define CONTROL_SET_PREFIX "CONTROL SET"

/**
 * @brief Report the complete volatile configuration used by the next ARM.
 *
 * Called internally by control_usb_command_handler() for GET, SET, and
 * DEFAULTS. It performs one formatted synchronous write through @p io and
 * retains no pointers.
 *
 * @param io Valid transport response operations.
 * @param command Protocol response name without spaces.
 * @param configuration Valid configuration to serialize with explicit units in
 * the protocol field names.
 */
static void send_configuration(const esp_timeseries_usb_command_io_t *io,
                               const char *command,
                               const motor_controller_config_t *configuration) {
  /* Serialization block: one line makes the reply atomic at protocol level. */
  io->sendf("OK command=%s protocol=%u volatile=1 apply=next_arm "
            "kp=%.9g ki=%.9g kd=%.9g reference_step_period_s=%.9g\n",
            command, CONTROL_USB_PROTOCOL_VERSION, configuration->kp,
            configuration->ki, configuration->kd,
            configuration->reference_step_period_s);
}

/**
 * @brief Parse four positional CONTROL SET values and reject trailing text.
 *
 * Called internally only by control_usb_command_handler(). Parsing establishes
 * syntax; motor_controller_config_is_valid() separately establishes numerical
 * ranges.
 *
 * @param command Complete NUL-terminated command line.
 * @param configuration Destination written only when four floats are present.
 * @return true for exactly four values after the CONTROL SET prefix.
 */
static bool parse_set_command(const char *command,
                              motor_controller_config_t *configuration) {
  /* Namespace block: require the exact prefix followed by a separator. */
  const size_t prefix_length = strlen(CONTROL_SET_PREFIX);
  if (strncasecmp(command, CONTROL_SET_PREFIX, prefix_length) != 0 ||
      command[prefix_length] != ' ') {
    return false;
  }

  /* Field block: the fifth conversion detects otherwise ignored extra text. */
  char trailing = '\0';
  return sscanf(command + prefix_length, " %f %f %f %f %c", &configuration->kp,
                &configuration->ki, &configuration->kd,
                &configuration->reference_step_period_s, &trailing) == 4;
}

/**
 * @brief Dispatch one volatile CONTROL command from the USB transport.
 *
 * Called by application_usb_command_handler(). See the public header for the
 * protocol, ownership, and execution-context contract.
 */
bool control_usb_command_handler(const char *command,
                                 const esp_timeseries_usb_command_io_t *io,
                                 void *context) {
  /* Dispatch block: decline commands outside the complete CONTROL token. */
  if (command == NULL || io == NULL ||
      strncasecmp(command, "CONTROL", strlen("CONTROL")) != 0 ||
      (command[strlen("CONTROL")] != '\0' &&
       command[strlen("CONTROL")] != ' ')) {
    return false;
  }

  /* Context block: CONTROL is recognized, so missing application state is
   * answered here instead of being offered to another extension. */
  control_usb_command_context_t *control_context =
      (control_usb_command_context_t *)context;
  if (control_context == NULL || control_context->configuration == NULL) {
    io->send_error("CONTROL", ESP_ERR_INVALID_STATE,
                   "configuration_unavailable");
    return true;
  }

  /* Read/default blocks: reply with the complete resulting configuration. */
  if (strcasecmp(command, "CONTROL GET") == 0) {
    send_configuration(io, "CONTROL_GET", control_context->configuration);
    return true;
  }

  if (strcasecmp(command, "CONTROL DEFAULTS") == 0) {
    motor_controller_get_default_config(control_context->configuration);
    send_configuration(io, "CONTROL_DEFAULTS", control_context->configuration);
    return true;
  }

  /* Set block: parse and validate a temporary object before the single
   * assignment, preventing a partially updated live configuration. */
  motor_controller_config_t candidate = {0};
  if (!parse_set_command(command, &candidate)) {
    io->send_error("CONTROL_SET", ESP_ERR_INVALID_ARG,
                   "expected_kp_ki_kd_period_s");
  } else if (!motor_controller_config_is_valid(&candidate)) {
    io->send_error("CONTROL_SET", ESP_ERR_INVALID_ARG,
                   "parameter_out_of_range");
  } else {
    *control_context->configuration = candidate;
    send_configuration(io, "CONTROL_SET", control_context->configuration);
  }
  return true;
}
