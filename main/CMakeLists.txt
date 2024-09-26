idf_component_register(SRCS "netAPP.c" "main.c"
                            "esp_hidd_prf_api.c"
                            "hid_dev.c"
                            "hid_device_le_prf.c"
                            "bleAPP.c"
                            "nfcAPP.c"
                    INCLUDE_DIRS ".")

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-const-variable)
