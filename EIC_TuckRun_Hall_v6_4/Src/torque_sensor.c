#include <stdint.h>
#include "main.h"
#include "mc_uart_protocol.h"
#include "torque_sensor.h"

#include <stdio.h>

/* ====== Torque sensor 解析函式 ====== */
// 把 7-byte raw data 轉成溫度 / Nm / RPM 等
void Torque_ParsePacket(const uint8_t *data, TorquePacket_t *out)
{
    // printf("Torque_PP\n");
    for (int i = 0; i < 7; ++i) {
        out->raw[i] = data[i];
        // printf(" %d", data[i]);
    }
    // printf("\n");

    // 溫度：資料 2 高 6 bits
    uint8_t temp_raw = (data[1] >> 2) & 0x3F;
    out->temp_c = 71 + temp_raw;  // 0~63 -> 71~134℃

    // 力矩：資料 2 低 2 bits + 資料 3
    uint16_t torque_high = data[1] & 0x03;
    uint16_t torque_low  = data[2];
    uint16_t torque      = (torque_high << 8) | torque_low;
    out->torque_raw      = torque;

    // printf("data %d, %d\n", data[1], data[2]);

    float v_torque = ((float)torque / 1024.0f) * 5.0f;
    out->torque_voltage  = v_torque;

    const uint16_t zero_offset = 143;
    if (torque >= zero_offset) {
        float nm = (v_torque - 0.7f) / 0.035f;
        if (nm < 0.0f) nm = 0.0f;
        out->nm = nm;
    } else {
        out->nm = 0.0f;
    }
    // printf("T: %d\n", (int)(out->nm*100));
    
    // 資料4：bit7=反轉, bit0~6=踏平累加值
    out->flywheel_reverse = (data[3] >> 7) & 0x01;
    out->pedal_count      = data[3] & 0x7F;

    // 資料5、6：rpm pulse period
    uint8_t spa = data[4];
    uint8_t spb = data[5];
    out->spa   = spa;
    out->spb   = spb;

    uint16_t period = ((spa & 0x7F) << 8) | spb;
    out->motor_rpm_pulse_period = period;
    out->spa7_flag = (spa >> 7) & 0x01;

    if (period == 4500 || period == 0) {
        out->rpm = 0.0f;
    } else {
        out->rpm = 60.0f / (period * 0.000205f * 9.0f);
    }

    out->checksum = data[6];
}



