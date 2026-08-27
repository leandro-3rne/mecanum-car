idf_component_register(
    SRCS "main.c" "remote_control.c" "joysticks.c"
    INCLUDE_DIRS "."
    REQUIRES esp_wifi nvs_flash esp_adc
)
