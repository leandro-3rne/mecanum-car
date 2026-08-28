#pragma once

#include "esp_err.h"

typedef struct
{
    int ir1;
    int ir2;
    int ir3;
    int ir4;
    int ir5;
} line_sensor_values_t;

esp_err_t line_follow_init(void);
esp_err_t line_follow_read(line_sensor_values_t *values);
void line_follow_stop(void);
void line_follow_pid(line_sensor_values_t *values);
void line_follow_start(void);