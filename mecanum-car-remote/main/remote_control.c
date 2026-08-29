#include "remote_control.h"

#include <stdio.h>
#include <string.h>

#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_err.h"


//MAC-ADRESSE ---------------------------------------
static const uint8_t mecanum_mac[6] = {
    0xB4, 0xBF, 0xE9,
    0xC8, 0x70, 0x48
};


//WIFI-INIT
void wifi_init(void)
{
    nvs_flash_init();

    esp_netif_init();
    esp_event_loop_create_default();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();

    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
}


//SEND-CALLBACK
static void send_callback(
    const wifi_tx_info_t *tx_info,
    esp_now_send_status_t status
)
{
    printf(
        "Delivery: %s\n",
        status == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAIL"
    );
}


//ESP-NOW-INIT
void espnow_init(void)
{
    esp_now_init();

    esp_now_peer_info_t peer = {0};

    memcpy(peer.peer_addr, mecanum_mac, 6);
    peer.channel = 1;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;

    ESP_ERROR_CHECK(esp_now_add_peer(&peer));
    esp_now_register_send_cb(send_callback);
}


//REMOTE-CONTROL-SEND
void remote_control_send(const RemoteCommand *command)
{
    esp_now_send(mecanum_mac, (const uint8_t *)command, sizeof(*command));
}
