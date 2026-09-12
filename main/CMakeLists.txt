idf_component_register(SRCS "main.c"
                            "angle_lut_usb_commands.c"
                            "control_usb_commands.c"
                            "engine_angle_kalman.c"
                            "motor_controller.c"
                            "realtime_loop.c"
                            "realtime_telemetry.c"
                       INCLUDE_DIRS ""
                       REQUIRES esp-mt6701
                                esp_angle_lut
                                esp_pid
                                esp_rt_diagnostics
                                esp_rt_diagnostics_reporter
                                esp_timeseries_recorder
                                esp_timeseries_usb_transport
                                kalman-filter-c
                                esp-engine-driver
                                esp_driver_gptimer
                                esp_timer
                                nvs_flash)
