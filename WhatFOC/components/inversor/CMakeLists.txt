idf_component_register(SRCS "inversor.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_driver_gpio esp_driver_mcpwm)
