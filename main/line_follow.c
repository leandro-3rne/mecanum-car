#include "line_follow.h"

#include <stdbool.h>

#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
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
     -2.0f,
     -1.0f,
     0.0f,
    1.0f,
    2.0f
};


// =====================================================
// PIN / ADC CONFIG
// =====================================================

// Deine Verkabelung:
//
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


// =====================================================
// INIT
// =====================================================

esp_err_t line_follow_init(void)
{
    if (adc_handle != NULL)
    {
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

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize ADC1");
        return err;
    }


    adc_oneshot_chan_cfg_t channel_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };


    err = adc_oneshot_config_channel(
        adc_handle,
        IR1_CHANNEL,
        &channel_config
    );

    if (err != ESP_OK) return err;


    err = adc_oneshot_config_channel(
        adc_handle,
        IR2_CHANNEL,
        &channel_config
    );

    if (err != ESP_OK) return err;


    err = adc_oneshot_config_channel(
        adc_handle,
        IR3_CHANNEL,
        &channel_config
    );

    if (err != ESP_OK) return err;


    err = adc_oneshot_config_channel(
        adc_handle,
        IR4_CHANNEL,
        &channel_config
    );

    if (err != ESP_OK) return err;


    err = adc_oneshot_config_channel(
        adc_handle,
        IR5_CHANNEL,
        &channel_config
    );

    if (err != ESP_OK) return err;


    ESP_LOGI(TAG, "ADC channels initialized");

    return ESP_OK;
}


// =====================================================
// READ
// =====================================================

esp_err_t line_follow_read(line_sensor_values_t *values)
{
    if (adc_handle == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (values == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }


    esp_err_t err;


    err = adc_oneshot_read(
        adc_handle,
        IR1_CHANNEL,
        &values->ir1
    );

    if (err != ESP_OK) return err;


    err = adc_oneshot_read(
        adc_handle,
        IR2_CHANNEL,
        &values->ir2
    );

    if (err != ESP_OK) return err;


    err = adc_oneshot_read(
        adc_handle,
        IR3_CHANNEL,
        &values->ir3
    );

    if (err != ESP_OK) return err;


    err = adc_oneshot_read(
        adc_handle,
        IR4_CHANNEL,
        &values->ir4
    );

    if (err != ESP_OK) return err;


    err = adc_oneshot_read(
        adc_handle,
        IR5_CHANNEL,
        &values->ir5
    );

    if (err != ESP_OK) return err;


    return ESP_OK;
}



float normalize_ir(int raw, int black, int white)
{
    float value =
        (float)(white - raw) /
        (float)(white - black);

    if (value < 0.0f) value = 0.0f;
    if (value > 1.0f) value = 1.0f;

    return value;
}

static void calibrate_white(void)
{
    line_sensor_values_t values;

    int sum[5] = {0};

    const int samples = 30;

    for (int n = 0; n < samples; ++n)
    {
        ESP_ERROR_CHECK(
            line_follow_read(&values)
        );

        sum[0] += values.ir1;
        sum[1] += values.ir2;
        sum[2] += values.ir3;
        sum[3] += values.ir4;
        sum[4] += values.ir5;

        vTaskDelay(pdMS_TO_TICKS(5));
    }

    for (int i = 0; i < 5; ++i)
    {
        white[i] = sum[i] / samples;

        if (white[i] < black[i] + 500)
        {
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


void line_follow_pid(line_sensor_values_t *values)
{

    int raw[5] = {
        values->ir1,
        values->ir2,
        values->ir3,
        values->ir4,
        values->ir5
    };


    float sensor[5];

    for (int i = 0; i < 5; ++i)
    {
        sensor[i] =
            normalize_ir(
                raw[i],
                black[i],
                white[i]
            );
    }


    float weighted_sum = 0.0f;
    float sensor_sum = 0.0f;

    for (int i = 0; i < 5; ++i)
    {
        weighted_sum += weights[i] * sensor[i];
        sensor_sum += sensor[i];
    }

    float error = 0.0f;

    if (sensor_sum > 0.05f)
    {
        error = weighted_sum / sensor_sum;
    }


    float vy = 0.5f;

    safety_command_received();
    drive(
        0.0f,
        vy,
        error
    );

    ESP_LOGI(
        TAG,
        "IR: %4d %4d %4d %4d %4d | error: %.2f",
        raw[0],
        raw[1],
        raw[2],
        raw[3],
        raw[4],
        error
    );


    vTaskDelay(
        pdMS_TO_TICKS(100)
    );
}

static TaskHandle_t line_follow_task_handle = NULL;
static bool line_follow_running = false;


static void line_follow_task(void *arg)
{
    line_sensor_values_t sensors;

    while (line_follow_running)
    {
        ESP_ERROR_CHECK(
            line_follow_read(&sensors)
        );

        line_follow_pid(&sensors);

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    line_follow_task_handle = NULL;

    vTaskDelete(NULL);
}

void line_follow_start(void)
{
    if (line_follow_running) {
        return;
    }

    calibrate_white();

    line_follow_running = true;
    motor_standby(true);

    xTaskCreate(
        line_follow_task,
        "line_follow_task",
        4096,
        NULL,
        5,
        &line_follow_task_handle
    );
}

void line_follow_stop(void)
{
    line_follow_running = false;

    drive(
        0.0f,
        0.0f,
        0.0f
    );
    motor_standby(false);
}