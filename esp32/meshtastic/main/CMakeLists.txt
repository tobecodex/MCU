idf_component_register(SRCS "main.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES display radio driver nvs_flash esp_driver_gpio)