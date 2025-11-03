idf_component_register(SRCS "radio.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES driver esp_driver_spi esp_driver_gpio)