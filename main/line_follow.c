#include "line_follow.h"

#include <stdbool.h>

#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "motor.h"
#include "safety.h"

static const char *TAG = "LINE_FOLLOW";

static int white[5] = {
    3950,
    3950,
    3950,
    3950,
    3950
};

static const int black[5] = {
    1850,
    1800,
    1750,
    2250,
    2200
};

static const float weights[5] = {
     2.5f,
     1.0f,
     0.0f,
    -1.0f,
    -2.5f
};

static const float Kp = 0.85f;
static const float Ki = 0.05f;
static const float Kd = 0.50f;

static const float LINE_FOLLOW_BASE_SPEED = 0.4f;
static const float LINE_FOLLOW_SETPOINT = 0.0f;
static const float LINE_FOLLOW_LINE_LOST_THRESHOLD = 0.15f;
static const float LINE_FOLLOW_INTEGRAL_MAX = 0.5f;
static const float LINE_FOLLOW_OUTPUT_MAX = 1.0f;
static const float LINE_LOST_STOP_DELAY_S = 0.5f;

static float pid_integral = 0.0f;
static float pid_last_error = 0.0f;
static int64_t pid_last_time_us = 0;
static int64_t line_lost_at_us = 0;


//IR-PINS -------------------------------------------
// IR1 -> GPIO36 = ADC1_CHANNEL_0
// IR2 -> GPIO39 = ADC1_CHANNEL_3
// IR3 -> GPIO34 = ADC1_CHANNEL_6
// IR4 -> GPIO35 = ADC1_CHANNEL_7
// IR5 -> GPIO32 = ADC1_CHANNEL_4

#define IR1_CHANNEL ADC_CHANNEL_0
#define IR2_CHANNEL ADC_CHANNEL_3
#define IR3_CHANNEL ADC_CHANNEL_6
#define IR4_CHANNEL ADC_CHANNEL_7
#define IR5_CHANNEL ADC_CHANNEL_4

static adc_oneshot_unit_handle_t adc_handle = NULL;
static bool line_follow_running = false;


//LINE-FOLLOW-INIT
esp_err_t line_follow_init(void)
{
    if (adc_handle != NULL) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    adc_oneshot_unit_init_cfg_t unit_config = {
        .unit_id = ADC_UNIT_1,
    };

    esp_err_t err = adc_oneshot_new_unit(
        &unit_config,
        &adc_handle
    );

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ADC1");
        return err;
    }

    adc_oneshot_chan_cfg_t channel_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };

    err = adc_oneshot_config_channel(adc_handle, IR1_CHANNEL, &channel_config);
    if (err != ESP_OK) return err;

    err = adc_oneshot_config_channel(adc_handle, IR2_CHANNEL, &channel_config);
    if (err != ESP_OK) return err;

    err = adc_oneshot_config_channel(adc_handle, IR3_CHANNEL, &channel_config);
    if (err != ESP_OK) return err;

    err = adc_oneshot_config_channel(adc_handle, IR4_CHANNEL, &channel_config);
    if (err != ESP_OK) return err;

    err = adc_oneshot_config_channel(adc_handle, IR5_CHANNEL, &channel_config);
    if (err != ESP_OK) return err;

    ESP_LOGI(TAG, "ADC channels initialized");

    return ESP_OK;
}


//LINE-FOLLOW-READ
esp_err_t line_follow_read(line_sensor_values_t *values)
{
    if (adc_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (values == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err;

    err = adc_oneshot_read(adc_handle, IR1_CHANNEL, &values->ir1);
    if (err != ESP_OK) return err;

    err = adc_oneshot_read(adc_handle, IR2_CHANNEL, &values->ir2);
    if (err != ESP_OK) return err;

    err = adc_oneshot_read(adc_handle, IR3_CHANNEL, &values->ir3);
    if (err != ESP_OK) return err;

    err = adc_oneshot_read(adc_handle, IR4_CHANNEL, &values->ir4);
    if (err != ESP_OK) return err;

    err = adc_oneshot_read(adc_handle, IR5_CHANNEL, &values->ir5);
    if (err != ESP_OK) return err;

    return ESP_OK;
}


//NORMALIZE-IR
static float normalize_ir(int raw, int black_val, int white_val)
{
    float value =
        (float)(white_val - raw) /
        (float)(white_val - black_val);

    if (value < 0.0f) value = 0.0f;
    if (value > 1.0f) value = 1.0f;

    return value;
}


//CALIBRATE-WHITE
static void calibrate_white(void)
{
    line_sensor_values_t values;

    int sum[5] = {0};

    const int samples = 30;

    for (int n = 0; n < samples; ++n) {
        ESP_ERROR_CHECK(line_follow_read(&values));

        sum[0] += values.ir1;
        sum[1] += values.ir2;
        sum[2] += values.ir3;
        sum[3] += values.ir4;
        sum[4] += values.ir5;

        vTaskDelay(pdMS_TO_TICKS(5));
    }

    for (int i = 0; i < 5; ++i) {
        white[i] = sum[i] / samples;

        if (white[i] < black[i] + 500) {
            ESP_LOGW(TAG, "Bad calibration on sensor %d", i);
            white[i] = 3950;
        }
    }

    ESP_LOGI(
        TAG,
        "White calibration: %d %d %d %d %d",
        white[0],
        white[1],
        white[2],
        white[3],
        white[4]
    );
}


//PID-RESET
static void line_follow_pid_reset(void)
{
    pid_integral = 0.0f;
    pid_last_error = 0.0f;
    pid_last_time_us = 0;
    line_lost_at_us = 0;
}


//CLAMP
static float clampf(float value, float min, float max)
{
    if (value < min) {
        return min;
    }

    if (value > max) {
        return max;
    }

    return value;
}


//LINE-FOLLOW-PID
static void line_follow_pid(line_sensor_values_t *values)
{
    int raw[5] = {
        values->ir1,
        values->ir2,
        values->ir3,
        values->ir4,
        values->ir5
    };

    float sensor[5];

    for (int i = 0; i < 5; ++i) {
        sensor[i] = normalize_ir(raw[i], black[i], white[i]);
    }

    float weighted_sum = 0.0f;
    float sensor_sum = 0.0f;

    for (int i = 0; i < 5; ++i) {
        weighted_sum += weights[i] * sensor[i];
        sensor_sum += sensor[i];
    }

    int64_t now_us = esp_timer_get_time();
    float dt = (pid_last_time_us == 0)
        ? 0.02f
        : (float)(now_us - pid_last_time_us) / 1000000.0f;
    pid_last_time_us = now_us;

    if (dt <= 0.0f || dt > 1.0f) {
        dt = 0.02f;
    }

    float measurement = 0.0f;
    bool line_visible = sensor_sum > LINE_FOLLOW_LINE_LOST_THRESHOLD;

    if (line_visible) {
        measurement = weighted_sum / sensor_sum;
    } else {
        pid_integral = 0.0f;
    }

    float error = LINE_FOLLOW_SETPOINT - measurement;

    float p_term = Kp * error;

    if (line_visible) {
        pid_integral += error * dt;
        pid_integral = clampf(
            pid_integral,
            -LINE_FOLLOW_INTEGRAL_MAX,
            LINE_FOLLOW_INTEGRAL_MAX
        );
    }

    float i_term = Ki * pid_integral;

    float d_term = 0.0f;
    if (line_visible) {
        float derivative = (error - pid_last_error) / dt;
        d_term = Kd * derivative;
    }

    pid_last_error = error;

    float omega = clampf(
        p_term + i_term + d_term,
        -LINE_FOLLOW_OUTPUT_MAX,
        LINE_FOLLOW_OUTPUT_MAX
    );

    float vy = LINE_FOLLOW_BASE_SPEED;

    if (line_visible) {
        line_lost_at_us = 0;
    } else if (line_lost_at_us == 0) {
        line_lost_at_us = now_us;
    } else {
        float lost_for_s = (float)(now_us - line_lost_at_us) / 1000000.0f;

        if (lost_for_s >= LINE_LOST_STOP_DELAY_S) {
            vy = 0.0f;
            omega = 0.0f;
        }
    }

    safety_command_received();
    drive(0.0f, vy, omega);

    ESP_LOGI(
        TAG,
        "IR: %4d %4d %4d %4d %4d | err: %.2f P: %.2f I: %.2f D: %.2f omega: %.2f",
        raw[0],
        raw[1],
        raw[2],
        raw[3],
        raw[4],
        error,
        p_term,
        i_term,
        d_term,
        omega
    );
}


//LINE-FOLLOW-TASK
static void line_follow_task(void *arg)
{
    line_sensor_values_t sensors;

    while (line_follow_running) {
        ESP_ERROR_CHECK(line_follow_read(&sensors));

        line_follow_pid(&sensors);

        vTaskDelay(pdMS_TO_TICKS(20));
    }

    vTaskDelete(NULL);
}


//LINE-FOLLOW-START
void line_follow_start(void)
{
    if (line_follow_running) {
        return;
    }

    calibrate_white();
    line_follow_pid_reset();

    line_follow_running = true;
    motor_standby(true);

    xTaskCreate(
        line_follow_task,
        "line_follow_task",
        4096,
        NULL,
        5,
        NULL
    );
}


//LINE-FOLLOW-STOP
void line_follow_stop(void)
{
    line_follow_running = false;
    line_follow_pid_reset();

    drive(0.0f, 0.0f, 0.0f);
    motor_standby(false);
}
