#include "remote_control.h"

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>

#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_err.h"

#include "motor.h"
#include "safety.h"

typedef struct {
    float vx;
    float vy;
    float omega;
} RemoteCommand;

static bool remote_initialized = false;


//RECEIVE-CALLBACK
static void receive_callback(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {

    if (len != sizeof(RemoteCommand)) return;

    RemoteCommand command;

    memcpy(&command, data, sizeof(command));

    //Safety
    safety_command_received();
    motor_standby(true);

    printf(
        "RX: vx=%.2f vy=%.2f omega=%.2f\n",
        command.vx,
        command.vy,
        command.omega
    );

    drive(command.vx, command.vy, command.omega);

}


//REMOTE-INIT
void remote_init(void) {

    if (remote_initialized) return;

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(receive_callback));

    remote_initialized = true;

    printf("ESP-NOW receiver initialized\n");
}


//REMOTE-START
void remote_start(void)
{
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();

    //Gleicher Kanal wie Fernbedienung
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

    if (!remote_initialized) {
        remote_init();
    }
}


//REMOTE-STOP
void remote_stop(void)
{
    drive(0.0f, 0.0f, 0.0f);
    motor_standby(false);

    if (remote_initialized) {
        esp_now_unregister_recv_cb();
        esp_now_deinit();

        remote_initialized = false;
    }

    esp_wifi_stop();
}
