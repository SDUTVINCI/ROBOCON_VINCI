/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    dma.c
  * @brief   This file provides code for the configuration
  *          of all the requested memory to memory DMA transfers.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "dma.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure DMA                                                              */
/*----------------------------------------------------------------------------*/

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * Enable DMA controller clock
  */
void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);

}

/* USER CODE BEGIN 2 */
// uint32_t dr16_uart_rx_data_handle(UART_HandleTypeDef *huart)
// {
//   if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))
//   {
//     /* clear idle it flag avoid idle interrupt all the time */
//     __HAL_UART_CLEAR_IDLEFLAG(huart);

//     /* clear DMA transfer complete flag */
//     __HAL_DMA_DISABLE(huart->hdmarx);

//     /* handle dbus data dbus_buf from DMA */
//     if ((DR16_RX_BUFFER_SIZE - huart->hdmarx->Instance->NDTR) == DR16_DATA_LEN)
//     {
//       if (dr16_rx_callback != NULL)
//       {
//         dr16_rx_callback(dr16_uart_rx_buff, DR16_DATA_LEN);
//       }

//       if (dr16_forword_callback != NULL)
//       {
//         dr16_forword_callback(dr16_uart_rx_buff, DR16_DATA_LEN);
//       }
//     }

//     /* restart dma transmission */
//     __HAL_DMA_SET_COUNTER(huart->hdmarx, DR16_RX_BUFFER_SIZE);
//     __HAL_DMA_ENABLE(huart->hdmarx);
//   }
//   return 0;
// }
/* USER CODE END 2 */

