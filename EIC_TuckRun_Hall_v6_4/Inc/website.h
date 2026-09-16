#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#include "power_control.h" // for PowerControlState_t

// 可依需要調整
#define UART3_RX_BUF_SIZE 256
#define WEBSITE_RB_SIZE   512
#define WEBSITE_LINE_MAX  512

typedef struct {
    int32_t vel_x100;      // km/h * 100
    int32_t torque_x100;   // Nm * 100
    int32_t curr_s16A;       // s16A
    int32_t power_avg_x100; // W * 100
    int32_t current_cmd_x100; // PI controller output for Iq current reference * 100
    int32_t motor_power_w_x100; // Motor electrical power in Watts * 100
    int32_t phase_voltage_s16V; // Phase voltage 
    int32_t iq_s16A;
    int32_t id_s16A;
    int32_t vq_s16V;
    int32_t vd_s16V;

    PowerControlState_t pc_state; // current power control state
} WebsiteTelemetry_t;

// 初始化（把 UART3 handle 給 website 模組）
void website_init(UART_HandleTypeDef *huart3);

// 在 main init 後啟動 UART3 ReceiveToIdle DMA（你把 DMA buffer 傳進來）
void website_start_rx_dma(uint8_t *dma_buf, uint16_t dma_len);

// 在 HAL_UARTEx_RxEventCallback(UART3) 裡呼叫：把這次收到的 bytes 推進 ring buffer
void website_on_rx_event(uint16_t size);

// 在 main while(1) 內一直呼叫：解析命令、更新參數、回 ACK
void website_process(void);

// 更新要回傳給 RPi 的 telemetry 內容
void website_update_telemetry(const WebsiteTelemetry_t *tel);

// 送出一次 telemetry（你可以每 100ms 呼叫一次）
void website_send_telemetry(void);