#pragma once

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "soc/gpio_num.h"
#include <stdbool.h>



// STBY
#define STBY GPIO_NUM_4

// STRUCT-MOTORS--------------------------------------
typedef struct {
    gpio_num_t in1;
    gpio_num_t in2;
    ledc_channel_t channel;
} Motor;

extern Motor FR;
extern Motor FL;
extern Motor RR;
extern Motor RL;

void motor_init(void);
void set_motor(Motor motor, float speed);
void drive(float vx, float vy, float omega);
void motor_standby(bool enabled);
