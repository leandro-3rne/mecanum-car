idf_component_register(SRCS "remote_control.c" "mode.c" "main.c" "motor.c" "wifi_control.c" "safety.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_driver_gpio esp_driver_ledc esp_wifi nvs_flash esp_http_server esp_netif esp_event esp_timer
                    EMBED_TXTFILES "controller.html")
