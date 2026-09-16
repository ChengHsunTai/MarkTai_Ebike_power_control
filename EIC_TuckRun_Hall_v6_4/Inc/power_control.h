// power_control.h
#ifndef POWER_CONTROL_H
#define POWER_CONTROL_H

#include <stdbool.h>


typedef enum {
    PC_STATE_STOP,
    PC_STATE_IDLE,
    PC_STATE_KICK_OFF,
    PC_STATE_SPEED_UP,
    PC_STATE_REGULAR,
    PC_STATE_OVERSPEED
} PowerControlState_t;

void power_control(void);
const char* PowerControlState_ToString(PowerControlState_t state); // 新增的輔助函式

extern volatile bool control_flag;

// 讓 website_send_telemetry 可以用
extern float wheel_vel;
extern float motor_current;
extern float tq_hum;
extern float human_power_avg;
extern float current_command_g;
extern PowerControlState_t g_power_control_state; // 新增的狀態變數

#endif