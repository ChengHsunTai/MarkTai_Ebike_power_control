#include <stdio.h>
#include "power_control.h"
#include "mc_uart_protocol.h"
#include "main.h"
#include "stm32g4xx_hal.h"
#include "torque_sensor.h"
#include "queue.h"
#include "mc_type.h"
#include "mc_api.h"
#include "power_control_params.h"

extern TorquePacket_t torque_packet;
float human_power_avg = 0.0f;
volatile bool control_flag = false;
float current_command_g = 0.0f; // Make it global and accessible
PowerControlState_t g_power_control_state = PC_STATE_STOP;

/* constants */
#define BIKE_WHEEL_RADIUS 0.343f
#define DIFF_TIME 0.1f
#define CONTROL_PERIOD_MS 100.0f     // 你原本 WINDOW_SIZE/50 暗示控制週期=50ms

static float integral_error = 0.0f;
static float prev_current_command = 0.0f; // for current slew rate limiting

// for memory of assist level when human temporarily stops pedaling but bike is still moving (e.g. at traffic light)
#define SPEED_UP_REUSE_WINDOW_MS 7000U
#define SPEED_UP_DEFAULT_CURRENT 8500.0f

static float record_assist = 0.0f;
static uint32_t idle_from_regular_tick_ms = 0;
static bool record_assist_valid = false;

// for idle decay
#define IDLE_DECAY_TIME_MS 500.0f   // IDLE 電流下降時間，可調，建議 500~1500 ms
///////////////////

Queue human_power_queue;
float human_power_sum = 0.0f;

float t_ms;
float tq_hum;
float wheel_vel;
float wheel_rpm;
float power_hum;
float motor_rpm;
float motor_current;

volatile bool hysteresis = false;
volatile uint32_t cruise_count = 0;

float prev_power_ref = 0.0f;
float set_power_ref = 0.0f;

const char* PowerControlState_ToString(PowerControlState_t state) {
    switch (state) {
        case PC_STATE_STOP:      return "STOP";
        case PC_STATE_IDLE:      return "IDLE";
        case PC_STATE_KICK_OFF:  return "KICK_OFF";
        case PC_STATE_SPEED_UP:  return "SPEED_UP";
        case PC_STATE_REGULAR:   return "REGULAR";
        case PC_STATE_OVERSPEED: return "OVERSPEED";
        default:                 return "UNKNOWN";
    }
}

void slew_rate_limit(){
    if (prev_power_ref < g_power_params.power_ref_w)
    {
        set_power_ref = prev_power_ref + 10.0f * CONTROL_PERIOD_MS/1000.0f; // 每秒增加10W
        if (set_power_ref > g_power_params.power_ref_w) {
            set_power_ref = g_power_params.power_ref_w;
        }
    } else if (prev_power_ref > g_power_params.power_ref_w) {
        set_power_ref = prev_power_ref - 10.0f * CONTROL_PERIOD_MS/1000.0f; // 每秒減少10W
        if (set_power_ref < g_power_params.power_ref_w) {
            set_power_ref = g_power_params.power_ref_w;
        }
    } else {
        set_power_ref = g_power_params.power_ref_w;
    }
    prev_power_ref = set_power_ref;
    
    
}

void current_slew_rate(qd_t iqdref){
    
    if (g_power_control_state == PC_STATE_OVERSPEED)
    {
        if (iqdref.q > prev_current_command)
        {
            
            current_command_g = prev_current_command + 2000.0f * CONTROL_PERIOD_MS/1000.0f; // 每秒增加2000A
            if (current_command_g > iqdref.q) {
                current_command_g = iqdref.q;
            }
        } else if (iqdref.q < prev_current_command) {
            current_command_g = prev_current_command - 2000.0f * CONTROL_PERIOD_MS/1000.0f; // 每秒減少2000A
            if (current_command_g < iqdref.q) {
                current_command_g = iqdref.q;
            }
        }else {
            current_command_g = iqdref.q;
        }
    }
    else if (g_power_control_state == PC_STATE_REGULAR){
        if ((iqdref.q - prev_current_command >= 5000) ||(prev_current_command - iqdref.q > 5000)){
            if (iqdref.q > prev_current_command)
            {
                current_command_g = prev_current_command + 20000.0f * CONTROL_PERIOD_MS/1000.0f; // 每秒增加3000A
                if (current_command_g > iqdref.q) {
                    current_command_g = iqdref.q;
                }
            } else if (iqdref.q < prev_current_command) {
                current_command_g = prev_current_command - 20000.0f * CONTROL_PERIOD_MS/1000.0f; // 每秒減少3000A
                if (current_command_g < iqdref.q) {
                    current_command_g = iqdref.q;
                }
            }else {
                current_command_g = iqdref.q;
            }  
        } 
        else {
            current_command_g = iqdref.q;
        }
    }
    else if (g_power_control_state == PC_STATE_SPEED_UP){
        if (iqdref.q > prev_current_command)
        {
            
            current_command_g = prev_current_command + 20000.0f * CONTROL_PERIOD_MS/1000.0f; // 每秒增加4000A
            if (current_command_g > iqdref.q) {
                current_command_g = iqdref.q;
            }
        } else if (iqdref.q < prev_current_command) {
            current_command_g = prev_current_command - 20000.0f * CONTROL_PERIOD_MS/1000.0f; // 每秒減少4000A
            if (current_command_g < iqdref.q) {
                current_command_g = iqdref.q;
            }
        }else {
            current_command_g = iqdref.q;
        }
    }
    else if (g_power_control_state == PC_STATE_IDLE || g_power_control_state == PC_STATE_KICK_OFF){
        if (iqdref.q > prev_current_command)
        {
            
            current_command_g = prev_current_command + 15000.0f * CONTROL_PERIOD_MS/1000.0f; // 每秒增加4000A
            if (current_command_g > iqdref.q) {
                current_command_g = iqdref.q;
            }
        } else if (iqdref.q < prev_current_command) {
            current_command_g = prev_current_command - 15000.0f * CONTROL_PERIOD_MS/1000.0f; // 每秒減少4000A
            if (current_command_g < iqdref.q) {
                current_command_g = iqdref.q;
            }
        }else {
            current_command_g = iqdref.q;
        }
    }
    prev_current_command = current_command_g;
    iqdref.q = (int16_t)current_command_g;
    iqdref.d = 0;
    MC_SetCurrentReferenceMotor1(iqdref);
}

void regulation()
{
    if (tq_hum < 1.2f ) // idle 
    {

        if (wheel_vel < 3.0f) {
            g_power_control_state = PC_STATE_STOP;
            integral_error = 0.0f; // reset integral term when fully stopped
            qd_t iqdref = {0};
            current_command_g = 0.0f;
            current_slew_rate(iqdref);
            prev_current_command = 0;
            (void)MC_StopMotor1();
            cruise_count = 0;
        } 
        else {
            if (integral_error > 20.0f) integral_error -= 20.0f;
            else integral_error = 0.0f;

            if (cruise_count < 2){
                cruise_count++;
                return;
                
            } 
            else{
                if (g_power_control_state == PC_STATE_REGULAR)
                {
                    record_assist = current_command_g;

                    if (record_assist > 12500.0f) record_assist = 12500.0f;
                    if (record_assist < 0.0f)     record_assist = 0.0f;

                    idle_from_regular_tick_ms = HAL_GetTick();
                    record_assist_valid = true;
                }
                g_power_control_state = PC_STATE_IDLE;
                integral_error = 0.0f; // reset integral term when fully stopped
                qd_t iqdref = {0};
                current_command_g = 0.0f;
                iqdref.q = (int16_t)current_command_g;
                iqdref.d = 0;
                current_slew_rate(iqdref);
                cruise_count = 0;
            }
                
        }

        return;
    }
    else if (human_power_avg < 10.0f) // near zero human power but still moving, likely coasting -> cut motor
    {
        qd_t iqdref = {0};
        if (tq_hum >= 3.5f && wheel_vel < 5.0f) {
            g_power_control_state = PC_STATE_KICK_OFF;
            // current_command_g = 0.0f;
            (void)MC_StartMotor1();
            if (g_power_params.kp != 0)
            {
                iqdref.q = 10000; 
                current_command_g = 10000.0f;
                MC_SetCurrentReferenceMotor1(iqdref);
                prev_current_command = current_command_g;
            } else {
                current_command_g = 0.0f;
                MC_SetCurrentReferenceMotor1(iqdref);
            }
        }
        // else {
        //     g_power_control_state = PC_STATE_STOP;
        //     current_command_g = 0.0f;
        //     current_slew_rate(iqdref);
        //     (void)MC_StopMotor1();
        // }
        cruise_count = 0;
        if (integral_error > 20.0f) integral_error -= 20.0f;
        else integral_error = 0.0f;
    }
    else if ((wheel_vel > 18.0) || (human_power_avg >= set_power_ref)) // human_power_avg >= set_power_ref, normal regulation
    {        
        float error = 0;
        if (g_power_control_state != PC_STATE_REGULAR)
        {
            if(g_power_control_state != PC_STATE_IDLE){
                if (record_assist_valid)
                {
                    uint32_t dt_ms = HAL_GetTick() - idle_from_regular_tick_ms;

                    if (dt_ms <= SPEED_UP_REUSE_WINDOW_MS)
                    {
                        current_command_g = record_assist;
                    }
                    else
                    {
                        record_assist_valid = false;
                    }
                }
            }
            integral_error = (current_command_g - error * g_power_params.kp)/g_power_params.ki;
        } 
        else
        {
            error = human_power_avg - set_power_ref;
        
            float qwin = g_power_params.quantized_window_w;
            if (qwin < 1.0f) qwin = 1.0f; // safety
            error = ((int)(error / qwin)) * qwin;

            integral_error += error * DIFF_TIME;

        }
        g_power_control_state = PC_STATE_REGULAR;
        
        if (integral_error > (12500.0/g_power_params.ki)) integral_error = (12500.0/g_power_params.ki);
        if(g_power_params.kp == 0){
            current_command_g = g_power_params.kp * error + g_power_params.ki * integral_error;
        }else{
            current_command_g = g_power_params.kp * error + g_power_params.ki * integral_error;
        }
        
        if (current_command_g > 12500.0f) current_command_g = 12500.0f;
        if (current_command_g < 0.0f) current_command_g = 0.0f;
        qd_t iqdref = {0};
        // float temp = current_command_g;
        // current_command_g = 0.0f;
        // current_slew_rate(iqdref);
        (void)MC_StartMotor1();
        // current_command_g = temp;
        iqdref.q = (int16_t)current_command_g;
        iqdref.d = 0;
        
        current_slew_rate(iqdref);
        // prev_current_command = current_command_g;
        cruise_count = 0;
    }

    else if (human_power_avg < set_power_ref) // below reference but not zero
    {
        // if (tq_hum <= 1.0f) {
        //     g_power_control_state = PC_STATE_IDLE;
        //     qd_t iqdref = {0};
        //     current_command_g = 0.0f;
        //     MC_SetCurrentReferenceMotor1(iqdref);

        //     if (integral_error > 20.0f) integral_error -= 20.0f;
        //     else integral_error = 0.0f;
        // }
        g_power_control_state = PC_STATE_SPEED_UP;
        qd_t iqdref = {0};
        current_command_g = 0.0f;
        // MC_SetCurrentReferenceMotor1(iqdref);
        (void)MC_StartMotor1();
        if (g_power_params.kp != 0)
        {
            float speed_up_current = SPEED_UP_DEFAULT_CURRENT;

            if (record_assist_valid)
            {
                uint32_t dt_ms = HAL_GetTick() - idle_from_regular_tick_ms;

                if (dt_ms <= SPEED_UP_REUSE_WINDOW_MS)
                {
                    if(record_assist > SPEED_UP_DEFAULT_CURRENT){
                        speed_up_current = record_assist;
                    } else{
                        speed_up_current = SPEED_UP_DEFAULT_CURRENT;
                    }
                    
                }
                else
                {
                    record_assist_valid = false;
                }
            }

            iqdref.q = (int16_t)speed_up_current;
            current_command_g = speed_up_current;
            current_slew_rate(iqdref);
        } else {
            current_command_g = 0.0f;
            iqdref.q = 0;
            MC_SetCurrentReferenceMotor1(iqdref);
            prev_current_command = 0;
        }
        cruise_count = 0;
        cruise_count = 0;
        if (integral_error > 20.0f) integral_error -= 20.0f;
        else integral_error = 0.0f;
    }

}
void power_control(void){
    t_ms = HAL_GetTick();
    tq_hum = torque_packet.nm;
    wheel_rpm = torque_packet.rpm;
    wheel_vel = (wheel_rpm * 2 * 3.14159f * 60.0f / 1000.0f) * BIKE_WHEEL_RADIUS; // km/h
    motor_rpm = MC_GetMecSpeedAverageMotor1()* 6; // Convert dHz to RPM
    motor_current = MC_GetPhaseCurrentAmplitudeMotor1();
    // printf("tq: %d\n", (int) tq_hum);

    power_hum = tq_hum * wheel_rpm * 2 * 3.14159f / 60.0f; // W


    // Update moving average queue
    queue_enqueue(&human_power_queue, power_hum);
    human_power_sum += power_hum;

    uint32_t win_samples = (uint32_t)((float)g_power_params.window_ms / CONTROL_PERIOD_MS);
    if (win_samples < 1) win_samples = 1;

    if (human_power_queue.size > win_samples)
    {
        float oldest_power;
        queue_dequeue(&human_power_queue, &oldest_power);
        human_power_sum -= oldest_power;
    }

    if (human_power_queue.size > 0) {
        human_power_avg = human_power_sum / (human_power_queue.size);
    } else {
        human_power_avg = 0.0f;
    }

    slew_rate_limit(); // Apply slew rate limit to power reference

    // vel limits（先保持常數）
    if (wheel_vel > 25.0f) // 超過25km/h且不在遲滯狀態，切斷動力
    {
        g_power_control_state = PC_STATE_OVERSPEED;
        qd_t iqdref = {0};
        current_command_g = 0.0f;
        current_slew_rate(iqdref); // 設置電流參考為 0
        hysteresis = true; // 進入遲滯狀態
        if (integral_error > 20.0f) integral_error -= 20.0f;
        else integral_error = 0.0f;
        return;
    } 
    else if (23.0f < wheel_vel && wheel_vel < 25.0f) // 低於30km/h且在遲滯狀態，恢復動力
    {
        if (hysteresis){
            g_power_control_state = PC_STATE_OVERSPEED;
            qd_t iqdref = {0};
            current_command_g = 0.0f;
            current_slew_rate(iqdref); // 設置電流參考為 0
            if (integral_error > 20.0f) integral_error -= 20.0f;
            else integral_error = 0.0f;
        } else {
            regulation();
            hysteresis = false;
        }
    }
    else{
        // current_command_g = 0.0f;
        regulation();
        hysteresis = false;
    }
    prev_power_ref = set_power_ref;
}