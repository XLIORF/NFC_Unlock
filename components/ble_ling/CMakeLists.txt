idf_component_register(SRCS "ble_ling.c" 
                            "hid/esp_hidd_prf_api.c"
                            "hid/hid_dev.c"
                            "hid/hid_device_le_prf.c"
                            "hid/hid_main.c"
                            "bleunlock/bleunlock.c"
                    INCLUDE_DIRS "include" "hid/include" "bleunlock/include"
                    REQUIRES bt nvs_flash)
