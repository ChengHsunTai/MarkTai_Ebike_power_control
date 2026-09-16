// power_params.c
#include "power_control_params.h"
#include <string.h>
#include <stdlib.h>

volatile PowerParams_t g_power_params;

void PowerParams_InitDefaults(void)
{
    g_power_params.power_ref_w        = 100.0f;
    g_power_params.window_ms          = 2000;
    g_power_params.quantized_window_w = 10.0f;
    g_power_params.kp                 = 75.0f;
    g_power_params.ki                 = 35.0f;
}

static bool in_range_f(float x, float lo, float hi) { return (x >= lo && x <= hi); }
static bool in_range_u32(uint32_t x, uint32_t lo, uint32_t hi) { return (x >= lo && x <= hi); }

bool PowerParams_SetByKey(const char *key, const char *value_str)
{
    if (!key || !value_str) return false;

    float vf = strtof(value_str, NULL);
    uint32_t vu = (uint32_t)strtoul(value_str, NULL, 10);

    if (strcmp(key, "POWER_REF_W") == 0) {
        if (!in_range_f(vf, 50.0f, 150.0f)) return false;
        g_power_params.power_ref_w = vf;
        return true;
    }
    if (strcmp(key, "WINDOW_MS") == 0) {
        if (!in_range_u32(vu, 100, 20000)) return false;
        g_power_params.window_ms = vu;
        return true;
    }
    if (strcmp(key, "QWIN_W") == 0) {
        if (!in_range_f(vf, 1.0f, 100.0f)) return false;
        g_power_params.quantized_window_w = vf;
        return true;
    }
    if (strcmp(key, "KP") == 0) {
        if (!in_range_f(vf, 0.0f, 2000.0f)) return false;
        g_power_params.kp = vf;
        return true;
    }
    if (strcmp(key, "KI") == 0) {
        if (!in_range_f(vf, 0.0f, 2000.0f)) return false;
        g_power_params.ki = vf;
        return true;
    }

    return false;
}