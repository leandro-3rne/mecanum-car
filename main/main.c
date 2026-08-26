#include "driver/gpio.h"
#include "soc/gpio_num.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "motor.h"
#include "wifi_control.h"
#include "safety.h"

//BUTTON (BOOT ESP32)
// #define BUTTON GPIO_NUM_0

void app_main(void)
{
    //Inits
    motor_init();
    safety_init();
    wifi_init();
    start_webserver();

    //Dircetion/Mode BUTTON
    // gpio_set_direction(BUTTON, GPIO_MODE_INPUT);
    // gpio_set_pull_mode(BUTTON, GPIO_PULLUP_ONLY);

    //Main-Loop
    while (1) {
        // int button = gpio_get_level(BUTTON);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
