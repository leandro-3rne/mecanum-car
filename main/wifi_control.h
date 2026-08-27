#pragma once

#include "esp_http_server.h"

void wifi_init(void);
void wifi_start();
void wifi_stop();
httpd_handle_t start_webserver(void);
