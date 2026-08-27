#pragma once

typedef struct {
    float vx;
    float vy;
    float omega;
} RemoteCommand;

void wifi_init(void);
void espnow_init(void);
void remote_control_send(const RemoteCommand *command);
