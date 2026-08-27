#include "safety.h"

#include "motor.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"


#define SAFETY_TIMEOUT_US 300000   // 300 ms


static int64_t last_command_time = 0;


void safety_command_received(void)
{
    last_command_time = esp_timer_get_time();
}


static void safety_task(void *arg)
{
    while (1)
    {
        int64_t now = esp_timer_get_time();

        if (now - last_command_time > SAFETY_TIMEOUT_US)
        {
            drive(0.0f, 0.0f, 0.0f);
            motor_standby(false);
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}


void safety_init(void)
{
    last_command_time = esp_timer_get_time();

    xTaskCreate(
        safety_task,
        "safety_task",
        2048,
        NULL,
        5,
        NULL
    );
}