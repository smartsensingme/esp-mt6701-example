idf_component_register(SRCS "esp_timeseries_usb_transport.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_angle_lut
                                esp_timeseries_recorder
                                esp_driver_usb_serial_jtag
                                esp_hw_support
                                freertos
                                log)
