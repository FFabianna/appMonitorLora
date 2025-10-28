idf_component_register(
    SRCS
        "gatewayUart.c"
        "main.c"
        "src/lora_uart/LoraUart.c"
        "src/leds/leds.c"
    INCLUDE_DIRS
        "."
        "../library/include"
        "src/lora_uart"
        "src/leds"
)

add_prebuilt_library(libdlms "../library/libdlms.a"
             PRIV_REQUIRES log)

target_link_libraries(${COMPONENT_LIB} INTERFACE libdlms)