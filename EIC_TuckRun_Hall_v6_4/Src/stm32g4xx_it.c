/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32g4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "power_control.h"
#include "stm32g4xx_hal.h"
#include "website.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include "mc_uart_protocol.h"
#include "torque_sensor.h"

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim6;
extern DMA_HandleTypeDef hdma_uart5_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
/* USER CODE BEGIN EV */
extern uint8_t torque_rx_buf[RX_TORQUE_BUFFER_SIZE];
extern uint8_t torque_data_buf[RX_TORQUE_BUFFER_SIZE];
extern volatile bool torque_new_data_flag;
extern volatile uint16_t torque_data_length;

extern uint8_t uart3_rx_buf[UART3_RX_BUF_SIZE];
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel3 global interrupt.
  */
void DMA1_Channel3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel3_IRQn 0 */

  /* USER CODE END DMA1_Channel3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
  /* USER CODE BEGIN DMA1_Channel3_IRQn 1 */

  /* USER CODE END DMA1_Channel3_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel4 global interrupt.
  */
void DMA1_Channel4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel4_IRQn 0 */

  /* USER CODE END DMA1_Channel4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_tx);
  /* USER CODE BEGIN DMA1_Channel4_IRQn 1 */

  /* USER CODE END DMA1_Channel4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel5 global interrupt.
  */
void DMA1_Channel5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel5_IRQn 0 */

  /* USER CODE END DMA1_Channel5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart5_tx);
  /* USER CODE BEGIN DMA1_Channel5_IRQn 1 */

  /* USER CODE END DMA1_Channel5_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel6 global interrupt.
  */
void DMA1_Channel6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel6_IRQn 0 */

  /* USER CODE END DMA1_Channel6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA1_Channel6_IRQn 1 */

  /* USER CODE END DMA1_Channel6_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt / USART3 wake-up interrupt through EXTI line 28.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles UART5 global interrupt / UART5 wake-up interrupt through EXTI line 35.
  */
void UART5_IRQHandler(void)
{
  /* USER CODE BEGIN UART5_IRQn 0 */

  /* USER CODE END UART5_IRQn 0 */
  HAL_UART_IRQHandler(&huart5);
  /* USER CODE BEGIN UART5_IRQn 1 */

  /* USER CODE END UART5_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC1 and DAC3 channel underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/* USER CODE BEGIN 1 */
/**
  * @brief  UART 接收事件回呼函式 (IDLE Line, Half Transfer, Full Transfer)
  * @note   這個函式是 HAL 庫中的弱定義函式，我們在這裡重新實現它。
  * @param  huart: UART handle
  * @param  Size: 接收到的數據長度
  * @retval None
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
      if (huart->Instance == USART1) {
        // 扭力感測器封包長度為 7 bytes，只在收到完整封包時才處理
        if (Size == 7) {
           memset(torque_data_buf, 0, RX_TORQUE_BUFFER_SIZE);
           memcpy(torque_data_buf, torque_rx_buf, Size);
           torque_new_data_flag = true;
        } else {
          // 收到無效長度的封包，不更新數據，保留上一筆成功值
          // 可以選擇在此處增加錯誤計數或日誌
        }
        // 無論封包是否有效，都重新啟動 DMA 接收
        HAL_UARTEx_ReceiveToIdle_DMA(huart, torque_rx_buf, RX_TORQUE_BUFFER_SIZE);
    } else if (huart->Instance == USART3) {
        // 移除多餘的 HAL_UART_DMAStop(huart); ReceiveToIdle_DMA 會自動處理
        // 把這次收到的資料推進 ring buffer（website.c 會用你設定的 dma_buf 指標）
        website_on_rx_event(Size);

        // *** 加入這行來閃爍 LED ***
        // HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);


        // 立刻重新掛回 ReceiveToIdle DMA
        HAL_UARTEx_ReceiveToIdle_DMA(huart, uart3_rx_buf, UART3_RX_BUF_SIZE);
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
       
    }
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Check if the interrupt comes from our timer (TIM2)
  if (htim->Instance == TIM6)
  {
    // do control
    // printf("enter timer6 intterrupt...\n");
    // printf("TS: %lu\n", HAL_GetTick());
    control_flag = true;
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        // Log the error code for debugging
        // huart->ErrorCode will contain flags like HAL_UART_ERROR_ORE, HAL_UART_ERROR_FE, etc.

        // 1. Stop DMA
        HAL_UART_DMAStop(huart);

        // 2. Clear all error flags
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);

        // 3. Clear RDR/FIFO
        while((huart->Instance->ISR & USART_ISR_RXNE_RXFNE) != 0)
        {
           volatile uint32_t temp = huart->Instance->RDR;
           (void)temp; // 防止編譯器優化掉讀取操作
        }

        // 4. Attempt to re-enable DMA reception.
        // For persistent errors, you might need a more robust recovery (e.g., DeInit/Init).
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, torque_rx_buf, RX_TORQUE_BUFFER_SIZE);

        // 5. Close HT interrupt
        if (huart1.hdmarx) {
           __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
        }
    }
    else if (huart->Instance == USART3)
    {
        // Log the error code for debugging
        // huart->ErrorCode will contain flags like HAL_UART_ERROR_ORE, HAL_UART_ERROR_FE, etc.

        // 1. Stop DMA
        HAL_UART_DMAStop(huart);

        // 2. Clear all error flags
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);

        // 3. Clear RDR/FIFO
        while((huart->Instance->ISR & USART_ISR_RXNE) != 0)
        {
           volatile uint32_t temp = huart->Instance->RDR;
           (void)temp; // 防止編譯器優化掉讀取操作
        }

        // 4. Attempt to re-enable DMA reception.
        // For persistent errors, you might need a more robust recovery (e.g., DeInit/Init).
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, uart3_rx_buf, UART3_RX_BUF_SIZE);

        // 5. Close HT interrupt
        if (huart3.hdmarx) {
           __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
        }
    }
}

/* USER CODE END 1 */
