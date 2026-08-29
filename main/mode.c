#include "mode.h"

#include "motor.h"

static ControlMode current_mode = MODE_WIFI;


//MODE-INIT
void mode_init(void) {
    current_mode = MODE_WIFI;
}


//MODE-GET
ControlMode mode_get(void) {
    return current_mode;
}


//MODE-NEXT
void mode_next(void) {
    drive(0.0f, 0.0f, 0.0f);
    motor_standby(false);

    current_mode = (current_mode + 1) % 3;
}
