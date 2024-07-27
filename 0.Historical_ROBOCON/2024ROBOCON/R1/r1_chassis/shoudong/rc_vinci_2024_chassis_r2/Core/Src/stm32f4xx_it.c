/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32f4xx_it.c
 * @brief   Interrupt Service Routines.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32f4xx_it.h"
#include "FreeRTOS.h"
#include "task.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
#include "hwt101ct_232.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#include "data_pack.h"


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// uart2
uint8_t rx_buf_uart4;



////****************************************//////////////
extern uint8_t L1S_Y1_rx_buffer[1];
static unsigned int gresult_y1 = 0;
static int fstatus_y1 = 0;
static unsigned int gdistance_y1 = 0;
unsigned int gdistance_Y1 = 0;
static int gbytes_y1 = 0;
///***********************///
////****************************************//////////////

extern uint8_t L1S_Y2_rx_buffer[1];
static unsigned int gresult_y2 = 0;
static int fstatus_y2 = 0;
static unsigned int gdistance_y2 = 0;
unsigned int gdistance_Y2 = 0;
static int gbytes_y2 = 0;



extern uint8_t L1S_X1_rx_buffer[1];
static unsigned int gresult_x1 = 0;
static int fstatus_x1 = 0;
static unsigned int gdistance_x1 = 0;
unsigned int gdistance_X1 = 0;
static int gbytes_x1 = 0;


extern uint8_t L1S_X2_rx_buffer[1];
static unsigned int gresult_x2 = 0;
static int fstatus_x2 = 0;
static unsigned int gdistance_x2 = 0;
unsigned int gdistance_X2 = 0;
static int gbytes_x2 = 0;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// extern uint8_t imu_buffer[1];
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
#if (INCLUDE_xTaskGetSchedulerState == 1 )
  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
  {
#endif /* INCLUDE_xTaskGetSchedulerState */
  xPortSysTickHandler();
#if (INCLUDE_xTaskGetSchedulerState == 1 )
  }
#endif /* INCLUDE_xTaskGetSchedulerState */
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles CAN1 RX0 interrupts.
  */
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */

  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */

  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
if (!fstatus_y2)
  {
    gresult_y2 = gresult_y2 | L1S_Y2_rx_buffer[0];
    if ((gresult_y2 & 0xffffff) == 0XB46904) 
    {
      fstatus_y2 = 1; // 找到帧头了
      gresult_y2 = 0;
    }
    else
    {
      gresult_y2 = gresult_y2 << 8;
    }
  }
  else
  {
    gbytes_y2++;
    gdistance_y2 = gdistance_y2 | L1S_Y2_rx_buffer[0];
    if (gbytes_y2 != 4)
    {
      gdistance_y2 = gdistance_y2 << 8;
    }
    else
    {
      gdistance_Y2 = gdistance_y2;
      gbytes_y2 = 0;
      gdistance_y2 = 0;
      fstatus_y2 = 0;
    }
  }
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
 
  HAL_UART_Receive_IT(&huart1, L1S_Y2_rx_buffer, 1);
  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  // 与执行机构主控通信
  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */
if (!fstatus_x2)
  {
    gresult_x2 = gresult_x2 | L1S_X2_rx_buffer[0];
    if ((gresult_x2 & 0xffffff) == 0XB46904) 
    {
      fstatus_x2 = 1; // 找到帧头了
      gresult_x2 = 0;
    }
    else
    {
      gresult_x2 = gresult_x2 << 8;
    }
  }
  else
  {
    gbytes_x2++;
    gdistance_x2 = gdistance_x2 | L1S_X2_rx_buffer[0];
    if (gbytes_x2 != 4)
    {
      gdistance_x2 = gdistance_x2 << 8;
    }
    else
    {
      gdistance_X2 = gdistance_x2;
      gbytes_x2 = 0;
      gdistance_x2 = 0;
      fstatus_x2 = 0;
    }
  }
  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

	HAL_UART_Receive_IT(&huart3, L1S_X2_rx_buffer, 1);
  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles UART4 global interrupt.
  */
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */
  ReceivefromMapan(rx_buf_uart4);
  /* USER CODE END UART4_IRQn 0 */
  HAL_UART_IRQHandler(&huart4);
  /* USER CODE BEGIN UART4_IRQn 1 */
  HAL_UART_Receive_IT(&huart4, &rx_buf_uart4, 1);
  /* USER CODE END UART4_IRQn 1 */
}

/**
  * @brief This function handles UART5 global interrupt.
  */
void UART5_IRQHandler(void)
{
  /* USER CODE BEGIN UART5_IRQn 0 */
if (!fstatus_x1)
  {
    gresult_x1 = gresult_x1 | L1S_X1_rx_buffer[0];
    if ((gresult_x1 & 0xffffff) == 0XB46904) 
    {
      fstatus_x1 = 1; // 找到帧头了
      gresult_x1 = 0;
    }
    else
    {
      gresult_x1 = gresult_x1 << 8;
    }
  }
  else
  {
    gbytes_x1++;
    gdistance_x1 = gdistance_x1 | L1S_X1_rx_buffer[0];
    if (gbytes_x1 != 4)
    {
      gdistance_x1 = gdistance_x1 << 8;
    }
    else
    {
      gdistance_X1 = gdistance_x1;
      gbytes_x1 = 0;
      gdistance_x1 = 0;
      fstatus_x1 = 0;
    }
  }
  /* USER CODE END UART5_IRQn 0 */
  HAL_UART_IRQHandler(&huart5);
  /* USER CODE BEGIN UART5_IRQn 1 */
HAL_UART_Receive_IT(&huart5, L1S_X1_rx_buffer, 1);
  /* USER CODE END UART5_IRQn 1 */
}

/**
  * @brief This function handles CAN2 RX0 interrupts.
  */
void CAN2_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN2_RX0_IRQn 0 */

  /* USER CODE END CAN2_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan2);
  /* USER CODE BEGIN CAN2_RX0_IRQn 1 */

  /* USER CODE END CAN2_RX0_IRQn 1 */
}

/**
  * @brief This function handles USART6 global interrupt.
  */
void USART6_IRQHandler(void)
{
  /* USER CODE BEGIN USART6_IRQn 0 */
  if (!fstatus_y1)
  {
    gresult_y1 = gresult_y1 | L1S_Y1_rx_buffer[0];
    if ((gresult_y1 & 0xffffff) == 0XB46904) // 帧头是B4 69 03
    {
      fstatus_y1 = 1; // 找到帧头了
      gresult_y1 = 0;
    }
    else
    {
      gresult_y1 = gresult_y1 << 8;
    }
  }
  else
  {
    gbytes_y1++;
    gdistance_y1 = gdistance_y1 | L1S_Y1_rx_buffer[0];
    if (gbytes_y1 != 4)
    {
      gdistance_y1 = gdistance_y1 << 8;
    }
    else
    {
      gdistance_Y1 = gdistance_y1;
      gbytes_y1 = 0;
      gdistance_y1 = 0;
      fstatus_y1 = 0;
    }
  }
  /* USER CODE END USART6_IRQn 0 */
  HAL_UART_IRQHandler(&huart6);
  /* USER CODE BEGIN USART6_IRQn 1 */
  HAL_UART_Receive_IT(&huart6, L1S_Y1_rx_buffer, 1);
  /* USER CODE END USART6_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
