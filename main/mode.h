#pragma once

typedef enum {
    MODE_WIFI,
    MODE_REMOTE,
    MODE_LINE_FOLLOW
} ControlMode;

void mode_init(void);
ControlMode mode_get(void);
void mode_next(void);
