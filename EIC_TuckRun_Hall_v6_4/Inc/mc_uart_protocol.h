#ifndef MC_UART_PROTOCOL_H
#define MC_UART_PROTOCOL_H

#include "main.h"
#define RX_BUF_SIZE 64
void UART_TEST(UART_HandleTypeDef *huart);
void UART_SendMCState(UART_HandleTypeDef *huart);
void UART_ParseAndExec(UART_HandleTypeDef *huart, const char *cmd);
const char* MC_ControlModeToString(MC_ControlMode_t mode);
const char* MCI_StateToString(MCI_State_t state);
const char* MCI_CommandStateToString(MCI_CommandState_t state);
const char* MC_FaultToString(uint16_t fault);

void UART_SendMotorInfo(UART_HandleTypeDef *huart);
void UART_SendMotorVector(UART_HandleTypeDef *huart);
#endif