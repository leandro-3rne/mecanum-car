#include "esp_adc/adc_oneshot.h"

#include "joysticks.h"


static adc_oneshot_unit_handle_t adc_handle;

void joysticks_init(void)
{

    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1
    };

    adc_oneshot_new_unit(&init_config, &adc_handle);

    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT
    };

    adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_4, &config); // GPIO33
    adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_5, &config); // GPIO32
    adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_6, &config); // GPIO35
    adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_7, &config); // GPIO34
}

static float normalize(int value, int center)
{
    float result;

    if (value >= center) {
        result = (float)(value - center) / (4095 - center);
    } else {
        result = (float)(value - center) / center;
    }

    if (result > 1.0f) result = 1.0f;
    if (result < -1.0f) result = -1.0f;

    if (result > -0.05f && result < 0.05f) {
        result = 0.0f;
    }

    return result;
}

RemoteCommand joysticks_read(void)
{
    int lx, ly, rx;

    adc_oneshot_read(adc_handle, ADC_CHANNEL_5, &lx);
    adc_oneshot_read(adc_handle, ADC_CHANNEL_4, &ly);
    adc_oneshot_read(adc_handle, ADC_CHANNEL_7, &rx);

    RemoteCommand command = {
        .vx = -normalize(lx, 1810),
        .vy = -normalize(ly, 1837),
        .omega = -normalize(rx, 1746)
    };

    return command;
}
