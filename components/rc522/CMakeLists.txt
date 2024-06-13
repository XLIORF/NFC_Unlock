idf_component_register(SRCS "src/rc522.c" "src/port.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_event driver
)
