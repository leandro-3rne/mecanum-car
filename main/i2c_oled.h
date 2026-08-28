#pragma once

#include "esp_err.h"
#include "mode.h"

esp_err_t screen_init(void);
esp_err_t screen_start(void);
esp_err_t screen_clear(void);
esp_err_t screen_set_mode(ControlMode mode);
esp_err_t screen_stop(void);
