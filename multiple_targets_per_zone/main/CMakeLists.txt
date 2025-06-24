idf_component_register(SRCS "main.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES driver freertos esp_driver_rmt led_strip )

include_directories($ENV{IDF_PATH}/components/esp_driver_rmt/include) 

target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-missing-field-initializers" "-Wall" "-Wno-error" "-Wunused-variable")
 