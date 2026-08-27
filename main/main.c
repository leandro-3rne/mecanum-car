#include "driver/gpio.h"
#include "mode.h"
#include "soc/gpio_num.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "motor.h"
#include "wifi_control.h"
#include "safety.h"
#include "remote_control.h"

//BUTTON (BOOT ESP32)
#define BUTTON GPIO_NUM_0

void app_main(void)
{
    //Inits
    motor_init();
    safety_init();
    mode_init();
    //Erster Modus (WIFI)
    wifi_init();
    wifi_start();

    uint8_t mac[6];

    //Dircetion/Mode BUTTON
    gpio_set_direction(BUTTON, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON, GPIO_PULLUP_ONLY);

    //Main-Loop
    int last_button = 1;

    while (1) {
        int button = gpio_get_level(BUTTON);
        

        if (last_button == 1 && button == 0) {

            ControlMode lastMode = mode_get();
            mode_next();
            ControlMode mode = mode_get();

            switch (lastMode) {

                case MODE_WIFI:
                    wifi_stop();
                    break;

                case MODE_REMOTE:
                    remote_stop();
                    break;

                case MODE_LINE_FOLLOW:
                    //line_follow_stop();
                    break;
            }
            
            switch (mode) {

                case MODE_WIFI:
                    wifi_start();
                    break;

                case MODE_REMOTE:
                    remote_start();
                    break;

                case MODE_LINE_FOLLOW:
                    // line_follow_start();
                    break;
            }
        }

        last_button = button;

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
