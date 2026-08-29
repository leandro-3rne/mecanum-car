#include "remote_control.h"
#include "joysticks.h"

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


//APP-MAIN
void app_main(void)
{
    wifi_init();
    espnow_init();
    joysticks_init();

    while (1) {

        RemoteCommand command = joysticks_read();

        remote_control_send(&command);

        printf("sent\n");

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
