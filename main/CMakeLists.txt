idf_component_register(SRCS "main.c" "engine_angle_kalman.c"
                       INCLUDE_DIRS ""
                       REQUIRES esp-mt6701 kalman-filter-c esp-engine-driver esp_timer)
