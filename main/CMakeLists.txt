idf_component_register(SRCS "main.c"
                            "engine_angle_kalman.c"
                            "motor_controller.c"
                            "realtime_loop.c"
                            "realtime_telemetry.c"
                       INCLUDE_DIRS ""
                       REQUIRES esp-mt6701
                                esp_rt_diagnostics
                                esp_timeseries_recorder
                                kalman-filter-c
                                esp-engine-driver
                                esp_driver_gptimer
                                esp_timer)
