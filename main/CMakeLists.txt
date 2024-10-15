idf_component_register(SRCS  "main.c"

                            "nfcAPP.c"
                    INCLUDE_DIRS ".")

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-const-variable)
