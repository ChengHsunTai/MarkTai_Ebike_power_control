#include "mc_uart_protocol.h"
#include "mc_api.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdint.h>

// --- Ring Buffer Configuration ---
#define UART_BUFFER_SIZE 256 // Choose a suitable size
typedef struct {
   uint8_t  usart_buf[UART_BUFFER_SIZE];
   uint16_t head; // Write position
   uint16_t tail; // Read position (managed by DMA)
} BUF_Struct_t;

static BUF_Struct_t Usart3_BS = {0};
static BUF_Struct_t Uart5_BS = {0};
extern UART_HandleTypeDef huart5;

static void UART_SendAck(UART_HandleTypeDef *huart, const char *cmd, bool ok);
static uint8_t CalculateChecksum(const char *data, int length);
int  usart_transmit(UART_HandleTypeDef *husart, uint8_t *ptr, int len, BUF_Struct_t *usart_bs);
// Using HAL_UART_Transmit_DMA to replace HAL_UART_Transmit(huart, (uint8_t*)buf, len, 10);

void UART_TEST(UART_HandleTypeDef *huart)
{
    HAL_UART_Transmit(huart, (uint8_t *)"Hello from UART!\r\n", 18, 100);
}
// dHz translates to RPM by multiplying with the dHz to RPM conversion factor : dHz * 0.1 * 60=6 

void UART_SendMCState(UART_HandleTypeDef *huart)
{
    MCI_State_t status = (MCI_State_t)MC_GetSTMStateMotor1();
    int cmd_state   = MC_GetCommandStateMotor1();
    int speed       = MC_GetMecSpeedAverageMotor1()* 6;
    //int speed       = MC_GetAverageMecSpeedMotor1_F()//UINT RPM;
    int current     = MC_GetPhaseCurrentAmplitudeMotor1();
    int occ_fault   = MC_GetOccurredFaultsMotor1();
    int cur_fault   = MC_GetCurrentFaultsMotor1();
    const char* status_str = MCI_StateToString(status);
    const char* cmd_state_str = MCI_CommandStateToString(cmd_state);
    const char* occ_str = MC_FaultToString(occ_fault);
    const char* cur_str = MC_FaultToString(cur_fault);

    char buf[256];

    int len = snprintf(buf, sizeof(buf),
        "$MCSTATE,STATUS,%s,CMD_STATE,%s,SPEED,%d,CURR,%d,OCC_FAULT,%s,CUR_FAULT,%s",
        status_str, cmd_state_str, speed, current, occ_str, cur_str );

        // Calculate XOR checksum
    uint8_t cs = CalculateChecksum(buf, len);
    len += snprintf(buf + len, sizeof(buf) - len, "*%02X\r\n", cs);

    usart_transmit(huart, (uint8_t*)buf, len, &Usart3_BS);
}

const char* MC_ControlModeToString(MC_ControlMode_t mode)
{
    switch (mode) {
        case MCM_OBSERVING_MODE:         return "OBSERVING";
        case MCM_OPEN_LOOP_VOLTAGE_MODE: return "OPEN_LOOP_VOLTAGE";
        case MCM_OPEN_LOOP_CURRENT_MODE: return "OPEN_LOOP_CURRENT";
        case MCM_SPEED_MODE:             return "SPEED";
        case MCM_TORQUE_MODE:            return "TORQUE";
        case MCM_PROFILING_MODE:         return "PROFILING";
        case MCM_SHORTED_MODE:           return "SHORTED";
        case MCM_POSITION_MODE:          return "POSITION";
        default:                         return "UNKNOWN";
    }
}
const char* MCI_StateToString(MCI_State_t state)
{
    switch (state) {
        case ICLWAIT:           return "ICLWAIT";
        case IDLE:              return "IDLE";
        case ALIGNMENT:         return "ALIGNMENT";
        case CHARGE_BOOT_CAP:   return "CHARGE_BOOT_CAP";
        case OFFSET_CALIB:      return "OFFSET_CALIB";
        case START:             return "START";
        case SWITCH_OVER:       return "SWITCH_OVER";
        case RUN:               return "RUN";
        case STOP:              return "STOP";
        case FAULT_NOW:         return "FAULT_NOW";
        case FAULT_OVER:        return "FAULT_OVER";
        case WAIT_STOP_MOTOR:   return "WAIT_STOP_MOTOR";
        default:                return "UNKNOWN";
    }
}
const char* MCI_CommandStateToString(MCI_CommandState_t state)
{
    switch (state) {
        case MCI_BUFFER_EMPTY:                   return "BUFFER_EMPTY";
        case MCI_COMMAND_NOT_ALREADY_EXECUTED:   return "NOT_EXECUTED";
        case MCI_COMMAND_EXECUTED_SUCCESSFULLY:  return "SUCCESS";
        case MCI_COMMAND_EXECUTED_UNSUCCESSFULLY:return "FAIL";
        default:                                 return "UNKNOWN";
    }
}
const char* MC_FaultToString(uint16_t fault)
{
    switch (fault) {
        case 0x0000: return "NONE";
        case 0x0001: return "DURATION";
        case 0x0002: return "OVER_VOLT";
        case 0x0004: return "UNDER_VOLT";
        case 0x0008: return "OVER_TEMP";
        case 0x0010: return "START_UP";
        case 0x0020: return "SPEED_FDBK";
        case 0x0040: return "OVER_CURR";
        case 0x0080: return "SW_ERROR";
        case 0x0400: return "DP_FAULT";
        default:     return "UNKNOWN";
    }
}
void UART_SendMotorInfo(UART_HandleTypeDef *huart)
{
    int speed_ref   = MC_GetMecSpeedReferenceMotor1()*6;
    int speed_avg   = MC_GetMecSpeedAverageMotor1()*6;
    int ramp_final  = MC_GetLastRampFinalSpeedMotor1()*6;
    MC_ControlMode_t ctrl_mode = MC_GetControlModeMotor1();
    const char* ctrl_mode_str = MC_ControlModeToString(ctrl_mode);
    int direction   = MC_GetImposedDirectionMotor1();
    int reliability = MC_GetSpeedSensorReliabilityMotor1();

    char buf[128];
    int len = snprintf(buf, sizeof(buf),
        "$MCINFO,SPEED_REF,%d,SPEED_AVG,%d,RAMP_FINAL,%d,CTRL_MODE,%s,DIR,%d,RELIABILITY,%d",
        speed_ref, speed_avg, ramp_final, ctrl_mode_str, direction, reliability);

    uint8_t cs = CalculateChecksum(buf, len);
    len += snprintf(buf + len, sizeof(buf) - len, "*%02X\r\n", cs);

    usart_transmit(huart, (uint8_t*)buf, len, &Usart3_BS);
}
void UART_SendMotorVector(UART_HandleTypeDef *huart)
{
    ab_t iab = MC_GetIabMotor1();
    alphabeta_t ialphabeta = MC_GetIalphabetaMotor1();
    qd_t iqd = MC_GetIqdMotor1();
    qd_t iqdref = MC_GetIqdrefMotor1();
    qd_t vqd = MC_GetVqdMotor1();
    alphabeta_t valphabeta = MC_GetValphabetaMotor1();
    int16_t el_angle = MC_GetElAngledppMotor1();
    int16_t te_ref = MC_GetTerefMotor1();

    char buf[192];
    int len = snprintf(buf, sizeof(buf),
        "$MCVEC,IA,%d,IB,%d,IALPHA,%d,IBETA,%d,ID,%d,IQ,%d,IDREF,%d,IQREF,%d,"
        "VD,%d,VQ,%d,VALPHA,%d,VBBETA,%d,EL_ANGLE,%d,TE_REF,%d",
        iab.a, iab.b,
        ialphabeta.alpha, ialphabeta.beta,
        iqd.d, iqd.q,
        iqdref.d, iqdref.q,
        vqd.d, vqd.q,
        valphabeta.alpha, valphabeta.beta,
        el_angle, te_ref
    );

    uint8_t cs = CalculateChecksum(buf, len);
    len += snprintf(buf + len, sizeof(buf) - len, "*%02X\r\n", cs);

    usart_transmit(huart, (uint8_t*)buf, len, &Usart3_BS);
}

static void UART_SendAck(UART_HandleTypeDef *huart, const char *cmd, bool ok)
{
    char buf[64];
    int len = snprintf(buf, sizeof(buf), "$ACK,%s,%s", cmd, ok ? "OK" : "ERR");
    uint8_t cs = CalculateChecksum(buf, len);
    len += snprintf(buf + len, sizeof(buf) - len, "*%02X\r\n", cs);
    usart_transmit(huart, (uint8_t*)buf, len, &Usart3_BS);
}

// Add a reusable XOR checksum calculation function
static uint8_t CalculateChecksum(const char *data, int length) {
    uint8_t checksum = 0;
    for (int i = 1; i < length; ++i) {
        checksum ^= data[i];
    }
    return checksum;
}

// Motor parameter constants (update as per user prompt)
#define MOTOR_KE           17.556f    // Vrms/kRPM
#define MOTOR_INERTIA      271.39e-6f // N.m.s^2 (converted from uN.m.s^2)
#define MAX_MOTOR_RPM      106        // dHz, 106 dHz = 636 rpm
#define BUS_CAPACITANCE    0.00066f   // Example: 660uF, adjust as needed
#define BUS_VOLTAGE_LIMIT  30.0f      // Example: 48V, adjust as needed
#define BUS_VOLTAGE_OVP    37.0f      // Example: 57V, adjust as needed
#define tlimit             50.0f    // Example: 50ms, adjust as needed
// float max_slope = (BUS_CAPACITANCE * BUS_VOLTAGE_LIMIT * (BUS_VOLTAGE_OVP - BUS_VOLTAGE_LIMIT)) / (MOTOR_INERTIA * MAX_MOTOR_RPM * tlimit);
// Update MAX_SPEED_RAMP_SLOPE as needed:
#define MAX_SPEED_RAMP_SLOPE 0.096f  // 0.096 dHz/ms


void MC_ProgramSpeedRampMotor1_Limited(UART_HandleTypeDef *huart, int16_t speed, uint16_t ramp_ms) {
    // Use the MAX_SPEED_RAMP_SLOPE macro defined above to limit the maximum slope
    // MAX_SPEED_RAMP_SLOPE is defined above with #define
    // int16_t current_speed = MC_GetMecSpeedAverageMotor1(); // in dHz
    // char buf[32];
    // snprintf(buf, sizeof(buf), "$CURRSPD,%d", current_speed * 6);
    // uint8_t cs = CalculateChecksum(buf, strlen(buf));
    // snprintf(buf + strlen(buf), sizeof(buf) - strlen(buf), "*%02X\r\n", cs);
    // HAL_UART_Transmit(huart, (uint8_t*)buf, strlen(buf), 10);
    // int16_t delta_speed = speed - current_speed; // in dHz
    // // Only limit ramp when decelerating (i.e., speed is decreasing)

    // if (delta_speed < 0) {
    //     float requested_slope = (float)abs(delta_speed) / (float)ramp_ms; // dHz/ms

    //     // If requested slope is too high, increase ramp_ms to limit the slope
    //     if (requested_slope > MAX_SPEED_RAMP_SLOPE) {
    //         ramp_ms = (uint16_t)((float)abs(delta_speed) / MAX_SPEED_RAMP_SLOPE);
    //         char ramp_str[16];
    //         snprintf(ramp_str, sizeof(ramp_str), "%u", ramp_ms);
    //         UART_SendAck(huart, ramp_str, true);
    //         if (ramp_ms == 0) ramp_ms = 1; // avoid zero ramp
    //     }
    // }
    MC_ProgramSpeedRampMotor1(speed, ramp_ms);
}


void UART_ParseAndExec(UART_HandleTypeDef *huart, const char *cmd)
{
    // ex $CTRL,START*CS\r\n
    // printf("Enter UART_ParseAndExec\n");
    // printf("%s\n", cmd);
    // printf("Parse: %s", cmd);
    if (strncmp(cmd, "$CMD,", 5) != 0) return;

    char tmp[32];
    strncpy(tmp, cmd + 5, sizeof(tmp) - 1);
    tmp[sizeof(tmp) - 1] = '\0';
    // Remove trailing \r or \n if present
    size_t tmplen = strlen(tmp);
    while (tmplen > 0 && (tmp[tmplen - 1] == '\r' || tmp[tmplen - 1] == '\n')) {
        tmp[--tmplen] = '\0';
    }
    bool ok = false;
    if (strcmp(tmp, "START") == 0) {
        ok = MC_StartMotor1();
        UART_SendAck(huart, "START", ok);
    } else if (strcmp(tmp, "STOP") == 0) {
        ok = MC_StopMotor1();
        UART_SendAck(huart, "STOP", ok);
    } else if (strncmp(tmp, "SPD,", 4) == 0) {
        int speed = 0, ramp = 0;
        if (sscanf(tmp + 4, "%d,%d", &speed, &ramp) == 2) {
            // Use the limited ramp function
            MC_StopSpeedRampMotor1();
            MC_ProgramSpeedRampMotor1_Limited(huart,(int16_t)(speed / 6), (uint16_t)ramp);
            ok = true;
        }
        char buf[64];
        int len = snprintf(buf, sizeof(buf), "$ACK,SPD,%s,SPEED,%d,RAMP,%d", ok ? "OK" : "ERR", speed, ramp);
        uint8_t cs = CalculateChecksum(buf, len);
        len += snprintf(buf + len, sizeof(buf) - len, "*%02X\r\n", cs);
        usart_transmit(huart, (uint8_t*)buf, len, &Usart3_BS);
    } else if (strncmp(tmp, "TORQUE,", 7) == 0) {
        // printf("Command: TORQUE,\n");
        int torque = 0, ramp = 0;
        if (sscanf(tmp + 7, "%d,%d", &torque, &ramp) == 2) {
            MC_ProgramTorqueRampMotor1(torque, ramp);
            char buf[64];
            int len = snprintf(buf, sizeof(buf), "$ACK,TORQUE,%s,TORQUE,%d,RAMP,%d", ok ? "OK" : "ERR", torque, ramp);
            uint8_t cs = CalculateChecksum(buf, len);
            len += snprintf(buf + len, sizeof(buf) - len, "*%02X\r\n", cs);
            usart_transmit(huart, (uint8_t*)buf, len, &Usart3_BS);
            ok = true;
        }
        UART_SendAck(huart, "TORQUE", ok);
    } else if (strncmp(tmp, "CURR,", 5) == 0) {
        // printf("Command: CURR,\n");
        qd_t iqdref = {0};
        if (sscanf(tmp + 5, "%hd,%hd", &iqdref.q, &iqdref.d) == 2) {
            MC_SetCurrentReferenceMotor1(iqdref);
            char buf[64];
            int len = snprintf(buf, sizeof(buf), "$ACK,CURR,%s,IQ,%d,ID,%d", ok ? "OK" : "ERR", iqdref.q, iqdref.d);
            uint8_t cs = CalculateChecksum(buf, len);
            len += snprintf(buf + len, sizeof(buf) - len, "*%02X\r\n", cs);
            usart_transmit(huart, (uint8_t*)buf, len, &Usart3_BS);
            ok = true;
        }
        UART_SendAck(huart, "CURR", ok);
    } else if (strcmp(tmp, "STOPSPEEDRAMP") == 0) {
        ok =MC_StopSpeedRampMotor1();
        UART_SendAck(huart, "STOPSPEEDRAMP", ok);
    } else if (strcmp(tmp, "STOPRAMP") == 0) {
        MC_StopRampMotor1();
        ok = true;
        UART_SendAck(huart, "STOPRAMP", ok);
        
    } else if (strcmp(tmp, "RAMPCOMPLETE") == 0) {
        ok = MC_HasRampCompletedMotor1();
        UART_SendAck(huart, "RAMPCOMPLETE", ok);
    } else if (strcmp(tmp, "CLEARIQD") == 0) {
        MC_Clear_IqdrefMotor1();
        ok = true;
        UART_SendAck(huart, "CLEARIQD", ok);
    } else if (strcmp(tmp, "ACKFAULT") == 0) {
        MC_AcknowledgeFaultMotor1();
        ok = true;
        UART_SendAck(huart, "ACKFAULT", ok);
    } else if (strcmp(tmp, "test") == 0) {
        ok = true;
        UART_SendAck(huart, "test", ok);
    }
    // MC API V6.4
    else if (strncmp(tmp, "SENSOR_SWITCH,", 14) == 0) {
        uint16_t  deltaAngleMargin = 0;
        if (sscanf(tmp + 14, "%hu", &deltaAngleMargin) == 1) {
            if (MC_SensorSwitchMotor1(deltaAngleMargin)){
                ok = true;
            }
            UART_SendAck(huart, "SENSOR_SWITCH", ok);
        }
    }
    else if (strncmp(tmp, "SENSOR_SET,HALL", 16) == 0) {
        if (MC_SensorSetMotor1(1000, (SpeednPosFdbk_Handle_t *)&HALL_M1)) {
            ok = true;
        }
        UART_SendAck(huart, "SENSOR_SET,HALL", ok);
    }
    else if (strncmp(tmp, "SENSOR_SET,STO_PLL", 18) == 0) {
        if (MC_SensorSetMotor1(1000, (SpeednPosFdbk_Handle_t *)&STO_PLL_M1)) {
            ok = true;
        }
        UART_SendAck(huart, "SENSOR_SET,STO_PLL", ok);
    }
}
    
/**
  * @brief Starts the DMA transfer if it's not already running.
  */
void uart_log_dma_start_transfer(UART_HandleTypeDef *husart, BUF_Struct_t *usart_bs)
{
    // We only start a new transfer if the UART is not already busy
    if (husart->Instance == USART3 || husart->Instance == UART5) {
       if (husart->gState == HAL_UART_STATE_READY)
       {
           // If head is behind tail, we have a wrap-around case
           if (usart_bs->head < usart_bs->tail)
           {
               // Transfer from tail to the end of the buffer
               uint16_t len = UART_BUFFER_SIZE - usart_bs->tail;
               HAL_UART_Transmit_DMA(husart, &usart_bs->usart_buf[usart_bs->tail], len);
               usart_bs->tail = 0; // Wrap tail around
           }
           else
           {
               // Normal case, transfer from tail to head
               uint16_t len = usart_bs->head - usart_bs->tail;
               if (len > 0)
               {
                   HAL_UART_Transmit_DMA(husart, &usart_bs->usart_buf[usart_bs->tail], len);
                   usart_bs->tail =usart_bs->head; // Move tail to where we've transmitted up to
               }
           }
       }
    }
}

/**
  * @brief  The _write syscall implementation. Copies data to the ring buffer.
  */
int  usart_transmit(UART_HandleTypeDef *husart, uint8_t *ptr, int len, BUF_Struct_t *usart_bs)
{
    if (husart->Instance == USART3){
        /* This functionality is now disabled as UART3 is used for website communication */
        /*
        // Copy data from the buffer to our ring buffer
           for (int i = 0; i < len; i++)
           {
               usart_bs->usart_buf[usart_bs->head] = *ptr++;
               usart_bs->head = (usart_bs->head + 1) % UART_BUFFER_SIZE;
           }
           // Start the DMA transfer if not already running
           uart_log_dma_start_transfer(husart, usart_bs);
        */
    }
    return len;
}

/**
  * @brief  The _write syscall implementation. Copies data to the ring buffer.
  */
int _write(int file, char *ptr, int len)
{
    // Copy data from the printf buffer to our ring buffer
    for (int i = 0; i < len; i++)
    {
        Uart5_BS.usart_buf[Uart5_BS.head] = *ptr++;
        Uart5_BS.head = (Uart5_BS.head + 1) % UART_BUFFER_SIZE;
    }

    // Start the DMA transfer if not already running
    uart_log_dma_start_transfer(&huart5, &Uart5_BS);
    
    return len;
}

// Implement the DMA transfer complete callback
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *husart) {
    // if (husart->Instance == USART3) { } // Disabled
    if (husart->Instance == UART5) { // Check for the correct UART
        // A DMA transfer has finished. Check if there's more data to send.
        uart_log_dma_start_transfer(husart, &Uart5_BS);
    }
}
