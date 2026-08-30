idf_component_register(SRCS "main.c"
                            "engine_angle_kalman.c"
                            "motor_controller.c"
                            "realtime_loop.c"
                       INCLUDE_DIRS ""
                       REQUIRES esp-mt6701
                                kalman-filter-c
                                esp-engine-driver
                                esp_driver_gptimer
                                esp_timer)
