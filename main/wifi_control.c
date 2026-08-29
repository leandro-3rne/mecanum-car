#include "wifi_control.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_http_server.h"
#include "esp_netif.h"

#include "motor.h"
#include "safety.h"

static bool wifi_initialized = false;
static httpd_handle_t server = NULL;


//WIFI-INIT
void wifi_init(void)
{
    if (wifi_initialized) return;

    //NVS
    nvs_flash_init();

    //Netzwerk-Stack
    esp_netif_init();

    //Event-System
    esp_event_loop_create_default();

    //Access-Point Netzwerkinterface
    esp_netif_create_default_wifi_ap();

    //WiFi-Treiber initialisieren
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    //Access Point konfigurieren
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = "Mecanum-Car",
            .password = "12345678",
            .channel = 1,
            .max_connection = 1,
            .authmode = WIFI_AUTH_WPA2_PSK
        },
    };

    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(WIFI_IF_AP, &wifi_config);

    wifi_initialized = true;
}


//WIFI-START
void wifi_start(void) {

    if (!wifi_initialized) {
        wifi_init();
    }

    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_start();

    if (server == NULL) {
        server = start_webserver();
    }
}


//WIFI-STOP
void wifi_stop(void) {
    if (server != NULL) {
        httpd_stop(server);
        server = NULL;
    }

    esp_wifi_stop();
}


//CONTROLLER-HTML
extern const unsigned char controller_html_start[]
    asm("_binary_controller_html_start");

extern const unsigned char controller_html_end[]
    asm("_binary_controller_html_end");


//CONTROLLER-HANDLER
esp_err_t controller_handler(httpd_req_t *req)
{
    size_t size =
        controller_html_end - controller_html_start;

    httpd_resp_set_type(req, "text/html");

    httpd_resp_send(
        req,
        (const char *)controller_html_start,
        size
    );

    return ESP_OK;
}


//DRIVE-HANDLER
esp_err_t drive_handler(httpd_req_t *req)
{
    char query[100];
    char value[16];

    float vx = 0.0f;
    float vy = 0.0f;
    float omega = 0.0f;

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {

        if (httpd_query_key_value(query, "vx", value, sizeof(value)) == ESP_OK)
            vx = atof(value);

        if (httpd_query_key_value(query, "vy", value, sizeof(value)) == ESP_OK)
            vy = atof(value);

        if (httpd_query_key_value(query, "omega", value, sizeof(value)) == ESP_OK)
            omega = atof(value);
    }

    printf("vx=%f, vy=%f, omega=%f\n", vx, vy, omega);

    //Safety
    safety_command_received();
    motor_standby(true);

    //Auto ansteuern
    drive(vx, vy, omega);

    httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}


//WEBSERVER-START
httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    httpd_start(&server, &config);

    httpd_uri_t root_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = controller_handler,
        .user_ctx = NULL
    };

    httpd_uri_t drive_uri = {
        .uri = "/drive",
        .method = HTTP_GET,
        .handler = drive_handler,
        .user_ctx = NULL
    };

    httpd_register_uri_handler(server, &root_uri);
    httpd_register_uri_handler(server, &drive_uri);

    return server;
}
