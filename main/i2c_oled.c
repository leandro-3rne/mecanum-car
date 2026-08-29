#include "i2c_oled.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

#define SCREEN_I2C_PORT I2C_NUM_0
#define SCREEN_SDA_GPIO 23
#define SCREEN_SCL_GPIO 22
#define SCREEN_ADDRESS  0x3C
#define SCREEN_I2C_FREQ 400000

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SCREEN_PAGES 8
#define FONT_WIDTH 5
#define FONT_HEIGHT 7
#define FONT_SCALE 2
#define FONT_CHAR_WIDTH ((FONT_WIDTH * FONT_SCALE) + 2)
#define FONT_CHAR_HEIGHT (FONT_HEIGHT * FONT_SCALE)

static const char *TAG = "SCREEN";

static i2c_master_bus_handle_t bus_handle = NULL;
static i2c_master_dev_handle_t screen_handle = NULL;
static uint8_t screen_buffer[SCREEN_WIDTH * SCREEN_PAGES];


//FONT ----------------------------------------------
static const uint8_t FONT_SPACE[5] = {0x00, 0x00, 0x00, 0x00, 0x00};

static const uint8_t FONT_A[5] = {0x7E, 0x11, 0x11, 0x11, 0x7E};
static const uint8_t FONT_E[5] = {0x7F, 0x49, 0x49, 0x49, 0x41};
static const uint8_t FONT_F[5] = {0x7F, 0x09, 0x09, 0x09, 0x01};
static const uint8_t FONT_I[5] = {0x00, 0x41, 0x7F, 0x41, 0x00};
static const uint8_t FONT_L[5] = {0x7F, 0x40, 0x40, 0x40, 0x40};
static const uint8_t FONT_M[5] = {0x7F, 0x02, 0x0C, 0x02, 0x7F};
static const uint8_t FONT_N[5] = {0x7F, 0x04, 0x08, 0x10, 0x7F};
static const uint8_t FONT_O[5] = {0x3E, 0x41, 0x41, 0x41, 0x3E};
static const uint8_t FONT_R[5] = {0x7F, 0x09, 0x19, 0x29, 0x46};
static const uint8_t FONT_T[5] = {0x01, 0x01, 0x7F, 0x01, 0x01};
static const uint8_t FONT_W[5] = {0x3F, 0x40, 0x38, 0x40, 0x3F};
static const uint8_t FONT_D[5] = {0x7F, 0x41, 0x41, 0x22, 0x1C};
static const uint8_t FONT_HYPHEN[5] = {0x08, 0x08, 0x08, 0x08, 0x08};


//LOW-LEVEL
static esp_err_t screen_write_command(uint8_t command)
{
    uint8_t data[2] = {
        0x00,
        command
    };

    return i2c_master_transmit(
        screen_handle,
        data,
        sizeof(data),
        1000
    );
}


static esp_err_t screen_write_data(const uint8_t *data, size_t length)
{
    if (length > 128) {
        return ESP_ERR_INVALID_SIZE;
    }

    uint8_t buffer[129];

    buffer[0] = 0x40;
    memcpy(&buffer[1], data, length);

    return i2c_master_transmit(
        screen_handle,
        buffer,
        length + 1,
        1000
    );
}


//CURSOR
static esp_err_t screen_set_cursor(uint8_t x, uint8_t page)
{
    esp_err_t err;

    err = screen_write_command(0x21);
    if (err != ESP_OK) return err;

    err = screen_write_command(x);
    if (err != ESP_OK) return err;

    err = screen_write_command(127);
    if (err != ESP_OK) return err;

    err = screen_write_command(0x22);
    if (err != ESP_OK) return err;

    err = screen_write_command(page);
    if (err != ESP_OK) return err;

    err = screen_write_command(7);

    return err;
}


//FONT-LOOKUP
static const uint8_t *screen_get_char(char c)
{
    switch (c) {
        case 'A': return FONT_A;
        case 'D': return FONT_D;
        case 'E': return FONT_E;
        case 'F': return FONT_F;
        case 'I': return FONT_I;
        case 'L': return FONT_L;
        case 'M': return FONT_M;
        case 'N': return FONT_N;
        case 'O': return FONT_O;
        case 'R': return FONT_R;
        case 'T': return FONT_T;
        case 'W': return FONT_W;
        case '-': return FONT_HYPHEN;
        case ' ': return FONT_SPACE;

        default:
            return FONT_SPACE;
    }
}


//FRAMEBUFFER
static void screen_buffer_clear(void)
{
    memset(screen_buffer, 0, sizeof(screen_buffer));
}

static void screen_buffer_set_pixel(uint8_t x, uint8_t y, int on)
{
    if (x >= SCREEN_WIDTH || y >= SCREEN_HEIGHT) {
        return;
    }

    uint8_t page = y / 8;
    uint8_t bit = y % 8;
    uint8_t *byte = &screen_buffer[(page * SCREEN_WIDTH) + x];

    if (on) {
        *byte |= (uint8_t)(1U << bit);
    } else {
        *byte &= (uint8_t)~(1U << bit);
    }
}

static esp_err_t screen_buffer_flush(void)
{
    for (int page = 0; page < SCREEN_PAGES; page++) {
        esp_err_t err = screen_set_cursor(0, page);
        if (err != ESP_OK) {
            return err;
        }

        err = screen_write_data(
            &screen_buffer[page * SCREEN_WIDTH],
            SCREEN_WIDTH
        );

        if (err != ESP_OK) {
            return err;
        }
    }

    return ESP_OK;
}


//DRAW-CHAR
static void screen_buffer_draw_char(uint8_t x, uint8_t y, char c)
{
    const uint8_t *font = screen_get_char(c);

    for (int col = 0; col < FONT_WIDTH; col++) {
        uint8_t column = font[col];

        for (int row = 0; row < FONT_HEIGHT; row++) {
            if ((column & (1U << row)) == 0) {
                continue;
            }

            for (int sy = 0; sy < FONT_SCALE; sy++) {
                for (int sx = 0; sx < FONT_SCALE; sx++) {
                    screen_buffer_set_pixel(
                        x + (col * FONT_SCALE) + sx,
                        y + (row * FONT_SCALE) + sy,
                        1
                    );
                }
            }
        }
    }
}


//DRAW-TEXT
static void screen_buffer_draw_text(uint8_t x, uint8_t y, const char *text)
{
    uint8_t cursor = x;

    while (*text) {
        screen_buffer_draw_char(cursor, y, *text);
        cursor = (uint8_t)(cursor + FONT_CHAR_WIDTH);
        text++;
    }
}


//SCREEN-INIT
esp_err_t screen_init(void)
{
    if (bus_handle != NULL) {
        return ESP_OK;
    }

    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = SCREEN_I2C_PORT,
        .sda_io_num = SCREEN_SDA_GPIO,
        .scl_io_num = SCREEN_SCL_GPIO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t err = i2c_new_master_bus(
        &bus_config,
        &bus_handle
    );

    if (err != ESP_OK) {
        return err;
    }

    i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SCREEN_ADDRESS,
        .scl_speed_hz = SCREEN_I2C_FREQ,
    };

    err = i2c_master_bus_add_device(
        bus_handle,
        &device_config,
        &screen_handle
    );

    if (err != ESP_OK) {
        i2c_del_master_bus(bus_handle);

        bus_handle = NULL;

        return err;
    }

    ESP_LOGI(TAG, "Initialized");

    return ESP_OK;
}


//SCREEN-START
esp_err_t screen_start(void)
{
    if (screen_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    vTaskDelay(pdMS_TO_TICKS(100));

    const uint8_t init_commands[] = {
        0xAE,
        0xD5, 0x80,
        0xA8, 0x3F,
        0xD3, 0x00,
        0x40,
        0x8D, 0x14,
        0x20, 0x00,
        0xA1,
        0xC8,
        0xDA, 0x12,
        0x81, 0xCF,
        0xD9, 0xF1,
        0xDB, 0x40,
        0xA4,
        0xA6,
        0xAF
    };

    for (size_t i = 0; i < sizeof(init_commands); i++) {
        esp_err_t err = screen_write_command(init_commands[i]);

        if (err != ESP_OK) {
            return err;
        }
    }

    ESP_LOGI(TAG, "Started");

    return screen_clear();
}


//SCREEN-CLEAR
esp_err_t screen_clear(void)
{
    if (screen_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    screen_buffer_clear();
    return screen_buffer_flush();
}


//SCREEN-SET-MODE
esp_err_t screen_set_mode(ControlMode mode)
{
    if (screen_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = screen_clear();

    if (err != ESP_OK) {
        return err;
    }

    const char *text;

    switch (mode) {
        case MODE_WIFI:
            text = "WIFI";
            break;

        case MODE_REMOTE:
            text = "REMOTE";
            break;

        case MODE_LINE_FOLLOW:
            text = "LINE-IR";
            break;

        default:
            text = "";
            break;
    }

    //Vertikal mittig
    uint8_t text_width = (uint8_t)(strlen(text) * FONT_CHAR_WIDTH);
    uint8_t x = (SCREEN_WIDTH - text_width) / 2;
    uint8_t y = (SCREEN_HEIGHT - FONT_CHAR_HEIGHT) / 2;

    screen_buffer_draw_text(x, y, text);

    return screen_buffer_flush();
}


//SCREEN-STOP
esp_err_t screen_stop(void)
{
    if (screen_handle == NULL) {
        return ESP_OK;
    }

    screen_write_command(0xAE);

    esp_err_t err = i2c_master_bus_rm_device(screen_handle);

    if (err != ESP_OK) {
        return err;
    }

    screen_handle = NULL;

    err = i2c_del_master_bus(bus_handle);

    if (err != ESP_OK) {
        return err;
    }

    bus_handle = NULL;

    ESP_LOGI(TAG, "Stopped");

    return ESP_OK;
}
