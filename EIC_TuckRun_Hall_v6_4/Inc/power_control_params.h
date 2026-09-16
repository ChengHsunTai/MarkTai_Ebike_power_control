// power_params.h
#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float    power_ref_w;        // POWER_REF
    uint32_t window_ms;          // WINDOW_SIZE (ms)
    float    quantized_window_w; // QUANTIZED_WINDOW
    float    kp;                 // K_P
    float    ki;                 // K_I
} PowerParams_t;

extern volatile PowerParams_t g_power_params;

void PowerParams_InitDefaults(void);

// website.c 解析後呼叫這個，用 key/value 更新參數（含範圍檢查）
bool PowerParams_SetByKey(const char *key, const char *value_str);