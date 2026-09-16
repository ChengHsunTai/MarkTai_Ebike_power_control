#include "website.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "power_control.h"
#include "power_control_params.h"   // 你提供的 params module

// ---------------- internal ----------------
static UART_HandleTypeDef *s_huart3 = NULL;

static uint8_t  *s_dma_buf = NULL;
static uint16_t  s_dma_len = 0;

static volatile uint16_t s_rb_head = 0;
static volatile uint16_t s_rb_tail = 0;
static uint8_t s_rb[WEBSITE_RB_SIZE];

static WebsiteTelemetry_t s_tel = {0};

// XOR checksum for body only (without '$' and without '*CS')
static uint8_t xor_checksum_body(const char *body)
{
    uint8_t cs = 0;
    for (const char *p = body; *p; ++p) cs ^= (uint8_t)(*p);
    return cs;
}

static void website_send_str(const char *s)
{
    if (!s_huart3 || !s) return;
    HAL_UART_Transmit(s_huart3, (uint8_t*)s, (uint16_t)strlen(s), 50);
    
}

static void send_packet_body(const char *body)
{
    char pkt[WEBSITE_LINE_MAX + 16];
    uint8_t cs = xor_checksum_body(body);
    snprintf(pkt, sizeof(pkt), "$%s*%02X\r\n", body, cs);
    website_send_str(pkt);
}

static void send_ack_ok_kv(const char *key, const char *val)
{
    char body[WEBSITE_LINE_MAX];
    snprintf(body, sizeof(body), "ACK,PCPARAM,OK,%s,%s", key, val);
    send_packet_body(body);
}

static void send_ack_err(const char *reason)
{
    char body[WEBSITE_LINE_MAX];
    snprintf(body, sizeof(body), "ACK,PCPARAM,ERR,%s", reason ? reason : "ERR");
    send_packet_body(body);
}

// ring buffer push (ISR context)
static void rb_push_bytes(const uint8_t *p, uint16_t n)
{
    for (uint16_t i = 0; i < n; i++) {
        uint16_t next = (uint16_t)((s_rb_head + 1) % WEBSITE_RB_SIZE);
        if (next == s_rb_tail) {
            // overflow: drop oldest
            s_rb_tail = (uint16_t)((s_rb_tail + 1) % WEBSITE_RB_SIZE);
        }
        s_rb[s_rb_head] = p[i];
        s_rb_head = next;
    }
}

// ring buffer pop one byte (main context)
static bool rb_pop(uint8_t *out)
{
    if (s_rb_tail == s_rb_head) return false;
    *out = s_rb[s_rb_tail];
    s_rb_tail = (uint16_t)((s_rb_tail + 1) % WEBSITE_RB_SIZE);
    return true;
}

// get one line ended by '\n' (main context), returns true if got a non-empty line
static bool rb_get_line(char *line_out, uint16_t max_len)
{
    uint16_t idx = 0;
    uint8_t ch;
    
    while (rb_pop(&ch)) {
        if (ch == '\r') continue;
        if (ch == '\n') {
            line_out[idx] = 0;
            return (idx > 0);
        }
        if (idx < (max_len - 1)) {
            line_out[idx++] = (char)ch;
        } else {
            // too long → flush to newline
            while (rb_pop(&ch)) {
                if (ch == '\n') break;
            }
            line_out[0] = 0;
            return false;
        }
    }
    return false;
}

// parse one command line (without \r\n)
static void handle_cmd_line(char *line)
{
    if (!line || !*line) return;

    // allow with or without '$'
    if (line[0] == '$') line++;

    // checksum optional: if '*' exists then verify XOR
    char *star = strchr(line, '*');
    if (star) {
        *star = 0;
        uint8_t recv = (uint8_t)strtoul(star + 1, NULL, 16);
        uint8_t calc = xor_checksum_body(line);
        if (recv != calc) { send_ack_err("BAD_CS"); return; }
    }

    char *save = NULL;
    char *t0 = strtok_r(line, ",", &save); // CMD
    char *t1 = strtok_r(NULL, ",", &save); // PCPARAM
    if (!t0 || !t1) { send_ack_err("BAD_FMT"); return; }

    if (strcmp(t0, "CMD") != 0) return;
    if (strcmp(t1, "PCPARAM") != 0) return;

    char *op = strtok_r(NULL, ",", &save); // SET / GET
    if (!op) { send_ack_err("BAD_FMT"); return; }

    if (strcmp(op, "SET") == 0) {
        char *key = strtok_r(NULL, ",", &save);
        char *val = strtok_r(NULL, ",", &save);
        if (!key || !val) { send_ack_err("BAD_FMT"); return; }

        bool ok = PowerParams_SetByKey(key, val);
        if (!ok) { send_ack_err("BAD_PARAM"); return; }

        send_ack_ok_kv(key, val);
        return;
    }

    if (strcmp(op, "GET") == 0) {
        // return current params (scaled integers to avoid float printf dependency)
        int pr_x100 = (int)(g_power_params.power_ref_w * 100.0f);
        int qw_x100 = (int)(g_power_params.quantized_window_w * 100.0f);
        int kp_x100 = (int)(g_power_params.kp * 100.0f);
        int ki_x100 = (int)(g_power_params.ki * 100.0f);

        char body[WEBSITE_LINE_MAX];
        snprintf(body, sizeof(body),
            "ACK,PCPARAM,OK,POWER_REF_x100,%d,WINDOW_MS,%lu,QWIN_x100,%d,KP_x100,%d,KI_x100,%d",
            pr_x100,
            (unsigned long)g_power_params.window_ms,
            qw_x100, kp_x100, ki_x100
        );
        send_packet_body(body);
        return;
    }

    send_ack_err("BAD_OP");
}

// ---------------- public API ----------------
void website_init(UART_HandleTypeDef *huart3)
{
    s_huart3 = huart3;
    s_rb_head = s_rb_tail = 0;
    memset(s_rb, 0, sizeof(s_rb));
}

void website_start_rx_dma(uint8_t *dma_buf, uint16_t dma_len)
{
    s_dma_buf = dma_buf;
    s_dma_len = dma_len;

    HAL_UARTEx_ReceiveToIdle_DMA(s_huart3, s_dma_buf, s_dma_len);
    __HAL_DMA_DISABLE_IT(s_huart3->hdmarx, DMA_IT_HT);
}

void website_on_rx_event(uint16_t size)
{
    if (!s_dma_buf || size == 0) return;
    if (size > s_dma_len) size = s_dma_len;

    rb_push_bytes(s_dma_buf, size);
}

void website_process(void)
{
    char line[WEBSITE_LINE_MAX];
    while (rb_get_line(line, sizeof(line))) {
        printf("Received line: %s\n", line);
        handle_cmd_line(line);
    }
}

void website_update_telemetry(const WebsiteTelemetry_t *tel)
{
    if (!tel) return;
    s_tel = *tel;
}

void website_send_telemetry(void)
{
    int pr_x100 = (int)(g_power_params.power_ref_w * 100.0f);
    int qw_x100 = (int)(g_power_params.quantized_window_w * 100.0f);
    int kp_x100 = (int)(g_power_params.kp * 100.0f);
    int ki_x100 = (int)(g_power_params.ki * 100.0f);

    const char *state_str = PowerControlState_ToString(s_tel.pc_state);

    char body[WEBSITE_LINE_MAX];
    snprintf(body, sizeof(body),
        "TEL,PCSTATE,t_ms,%lu,"
        "vel_x100,%ld,torque_x100,%ld,curr_s16A,%ld,power_avg_x100,%ld,"
        "motor_power_x100,%ld,current_cmd_x100,%ld,"
        "iq_s16A,%ld,id_s16A,%ld,"
        "state,%s,WINDOW_MS,%lu,POWER_REF_x100,%d,QWIN_x100,%d,KP_x100,%d,KI_x100,%d",
        (unsigned long)HAL_GetTick(),
        (long)s_tel.vel_x100,
        (long)s_tel.torque_x100,
        (long)s_tel.curr_s16A,
        (long)s_tel.power_avg_x100,
        (long)s_tel.motor_power_w_x100,
        (long)s_tel.current_cmd_x100,
        (long)s_tel.iq_s16A,
        (long)s_tel.id_s16A,
        state_str,
        (unsigned long)g_power_params.window_ms,
        pr_x100, qw_x100, kp_x100, ki_x100
    );

    send_packet_body(body);
}