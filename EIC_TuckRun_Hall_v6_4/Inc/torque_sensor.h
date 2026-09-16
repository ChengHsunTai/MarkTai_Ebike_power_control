

#ifndef TORQUE_SENSOR_H
#define TORQUE_SENSOR_H

#include <stdint.h>

#define RX_TORQUE_BUFFER_SIZE 16

// 解析後的封包結構
typedef struct {
    uint8_t  raw[7];
    uint8_t  temp_c;
    uint16_t torque_raw;
    float    torque_voltage;
    float    nm;
    uint8_t  flywheel_reverse;
    uint8_t  pedal_count;
    uint8_t  spa;
    uint8_t  spb;
    uint16_t motor_rpm_pulse_period;
    uint8_t  spa7_flag;
    float    rpm;
    uint8_t  checksum;
} TorquePacket_t;


void Torque_ParsePacket(const uint8_t *data, TorquePacket_t *out);


#endif