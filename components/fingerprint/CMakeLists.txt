idf_component_register(SRCS "fingerprint.c"
                        "driver/driver_as608.c"
                        "driver/driver_as608_interface.c"
                        "driver/driver_as608_basic.c"
                    INCLUDE_DIRS "include" "driver"
                    REQUIRES driver)
