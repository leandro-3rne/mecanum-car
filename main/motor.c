#include "motor.h"

#include <math.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "hal/gpio_types.h"
#include "hal/ledc_types.h"
#include "soc/gpio_num.h"

// PINs ----------------------------------------------
// Front
#define FRIN1 GPIO_NUM_13 // Front-Right-IN1
#define FRIN2 GPIO_NUM_33 // Front-Right-IN2

#define FLIN1 GPIO_NUM_25 // Front-Left-IN1
#define FLIN2 GPIO_NUM_26 // Front-Left-IN2

#define PWMFR GPIO_NUM_14 // PWMB-Front-Right
#define PWMFL GPIO_NUM_27 // PWMA-Front-Left

//Rear
#define RRIN1 GPIO_NUM_16 // Rear-Right-IN1
#define RRIN2 GPIO_NUM_17 // Rear-Right-IN2

#define RLIN1 GPIO_NUM_18 // Rear-Left-IN1
#define RLIN2 GPIO_NUM_19 // Rear-Left-IN2

#define PWMRR GPIO_NUM_5 // PWMB-Rear-Right
#define PWMRL GPIO_NUM_21 // PWMA-Rear-Left

Motor FR = {FRIN1, FRIN2, LEDC_CHANNEL_0};
Motor FL = {FLIN1, FLIN2, LEDC_CHANNEL_1};
Motor RR = {RRIN1, RRIN2, LEDC_CHANNEL_2};
Motor RL = {RLIN1, RLIN2, LEDC_CHANNEL_3};


// PWM-CONGIF ----------------------------------------
ledc_timer_config_t pwm_timer = {
    .speed_mode       = LEDC_LOW_SPEED_MODE,
    .duty_resolution  = LEDC_TIMER_8_BIT,
    .timer_num        = LEDC_TIMER_0,
    .freq_hz          = 20000,
    .clk_cfg          = LEDC_AUTO_CLK
};

ledc_channel_config_t pwm_fr = {
    .gpio_num   = PWMFR,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel    = LEDC_CHANNEL_0,
    .timer_sel  = LEDC_TIMER_0,
    .duty       = 0,
    .hpoint     = 0
};

ledc_channel_config_t pwm_fl = {
    .gpio_num   = PWMFL,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel    = LEDC_CHANNEL_1,
    .timer_sel  = LEDC_TIMER_0,
    .duty       = 0,
    .hpoint     = 0
};

ledc_channel_config_t pwm_rr = {
    .gpio_num   = PWMRR,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel    = LEDC_CHANNEL_2,
    .timer_sel  = LEDC_TIMER_0,
    .duty       = 0,
    .hpoint     = 0
};

ledc_channel_config_t pwm_rl = {
    .gpio_num   = PWMRL,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel    = LEDC_CHANNEL_3,
    .timer_sel  = LEDC_TIMER_0,
    .duty       = 0,
    .hpoint     = 0
};


//SAFETY-FUNCTION
void motor_standby(bool enabled)
{
    gpio_set_level(STBY, enabled);
}

//MOTOR-INIT
void motor_init(void)
{
    //Direction FR
    gpio_set_direction(FRIN1, GPIO_MODE_OUTPUT);
    gpio_set_direction(FRIN2, GPIO_MODE_OUTPUT);

    //Direction FL
    gpio_set_direction(FLIN1, GPIO_MODE_OUTPUT);
    gpio_set_direction(FLIN2, GPIO_MODE_OUTPUT);

    //Direction RR
    gpio_set_direction(RRIN1, GPIO_MODE_OUTPUT);
    gpio_set_direction(RRIN2, GPIO_MODE_OUTPUT);

    //Direction RL
    gpio_set_direction(RLIN1, GPIO_MODE_OUTPUT);
    gpio_set_direction(RLIN2, GPIO_MODE_OUTPUT);

    //Direction STBY
    gpio_set_direction(STBY, GPIO_MODE_OUTPUT);

    //PWM-Logik initialisieren
    ledc_timer_config(&pwm_timer);
    ledc_channel_config(&pwm_fr);
    ledc_channel_config(&pwm_fl);
    ledc_channel_config(&pwm_rr);
    ledc_channel_config(&pwm_rl);
}


//SET-MOTOR
void set_motor(Motor motor, float speed) {

    if (speed > 1.0f) speed = 1.0f;
    if (speed < -1.0f) speed = -1.0f;

    if (speed > 0) {
        gpio_set_level(motor.in1, 1);
        gpio_set_level(motor.in2, 0);
    }

    else if (speed < 0) {
        gpio_set_level(motor.in1, 0);
        gpio_set_level(motor.in2, 1);
    }

    else {
        gpio_set_level(motor.in1, 0);
        gpio_set_level(motor.in2, 0);
    }

    uint32_t duty = (uint32_t)(fabsf(speed) * 150);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, motor.channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, motor.channel);

}


//DRIVE-FUNCTION
void drive(float vx, float vy, float omega) {

    float fr = vy + vx - omega;
    float fl = vy + vx + omega;
    float rr = vy - vx - omega;
    float rl = vy - vx + omega;

    set_motor(FR, fr);
    set_motor(FL, fl);
    set_motor(RR, rr);
    set_motor(RL, rl);
}
