idf_component_register(SRCS "display.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES driver esp_driver_i2c esp_driver_gpio)