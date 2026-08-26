#pragma once

#include "esp_http_server.h"

void wifi_init(void);
httpd_handle_t start_webserver(void);
