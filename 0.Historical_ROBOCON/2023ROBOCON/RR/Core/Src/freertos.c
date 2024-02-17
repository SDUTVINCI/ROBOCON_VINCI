/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId CHASSISHandle;
osThreadId IMU_REQUESTHandle;
osThreadId CATCHHandle;
osThreadId SHOOTHandle;
osThreadId PUSHHandle;
osThreadId CAN2_CMDHandle;
osThreadId PNEUMATICHandle;
osMessageQId Catch_Target_QueueHandle;
osMessageQId Catch_Pre_QueueHandle;
osMessageQId Push_Go_QueueHandle;
osMessageQId Push_Origin_QueueHandle;
osSemaphoreId CP_Link_BinarySemHandle;
osSemaphoreId Pneumatic_BinarySemHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void chassis_task(void const * argument);
void imu_request_task(void const * argument);
void catch_task(void const * argument);
void shoot_task(void const * argument);
void push_task(void const * argument);
void can2_cmd_task(void const * argument);
void pneumatic_task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of CP_Link_BinarySem */
  osSemaphoreDef(CP_Link_BinarySem);
  CP_Link_BinarySemHandle = osSemaphoreCreate(osSemaphore(CP_Link_BinarySem), 1);

  /* definition and creation of Pneumatic_BinarySem */
  osSemaphoreDef(Pneumatic_BinarySem);
  Pneumatic_BinarySemHandle = osSemaphoreCreate(osSemaphore(Pneumatic_BinarySem), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of Catch_Target_Queue */
  osMessageQDef(Catch_Target_Queue, 32, fp32);
  Catch_Target_QueueHandle = osMessageCreate(osMessageQ(Catch_Target_Queue), NULL);

  /* definition and creation of Catch_Pre_Queue */
  osMessageQDef(Catch_Pre_Queue, 32, fp32);
  Catch_Pre_QueueHandle = osMessageCreate(osMessageQ(Catch_Pre_Queue), NULL);

  /* definition and creation of Push_Go_Queue */
  osMessageQDef(Push_Go_Queue, 32, fp32);
  Push_Go_QueueHandle = osMessageCreate(osMessageQ(Push_Go_Queue), NULL);

  /* definition and creation of Push_Origin_Queue */
  osMessageQDef(Push_Origin_Queue, 32, fp32);
  Push_Origin_QueueHandle = osMessageCreate(osMessageQ(Push_Origin_Queue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of CHASSIS */
  osThreadDef(CHASSIS, chassis_task, osPriorityNormal, 0, 128);
  CHASSISHandle = osThreadCreate(osThread(CHASSIS), NULL);

  /* definition and creation of IMU_REQUEST */
  osThreadDef(IMU_REQUEST, imu_request_task, osPriorityNormal, 0, 128);
  IMU_REQUESTHandle = osThreadCreate(osThread(IMU_REQUEST), NULL);

  /* definition and creation of CATCH */
  osThreadDef(CATCH, catch_task, osPriorityNormal, 0, 128);
  CATCHHandle = osThreadCreate(osThread(CATCH), NULL);

  /* definition and creation of SHOOT */
  osThreadDef(SHOOT, shoot_task, osPriorityNormal, 0, 128);
  SHOOTHandle = osThreadCreate(osThread(SHOOT), NULL);

  /* definition and creation of PUSH */
  osThreadDef(PUSH, push_task, osPriorityNormal, 0, 128);
  PUSHHandle = osThreadCreate(osThread(PUSH), NULL);

  /* definition and creation of CAN2_CMD */
  osThreadDef(CAN2_CMD, can2_cmd_task, osPriorityNormal, 0, 128);
  CAN2_CMDHandle = osThreadCreate(osThread(CAN2_CMD), NULL);

  /* definition and creation of PNEUMATIC */
  osThreadDef(PNEUMATIC, pneumatic_task, osPriorityNormal, 0, 128);
  PNEUMATICHandle = osThreadCreate(osThread(PNEUMATIC), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_chassis_task */
/**
  * @brief  Function implementing the CHASSIS thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_chassis_task */
__weak void chassis_task(void const * argument)
{
  /* USER CODE BEGIN chassis_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END chassis_task */
}

/* USER CODE BEGIN Header_imu_request_task */
/**
* @brief Function implementing the IMU_REQUEST thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_imu_request_task */
__weak void imu_request_task(void const * argument)
{
  /* USER CODE BEGIN imu_request_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END imu_request_task */
}

/* USER CODE BEGIN Header_catch_task */
/**
* @brief Function implementing the CATCH thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_catch_task */
__weak void catch_task(void const * argument)
{
  /* USER CODE BEGIN catch_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END catch_task */
}

/* USER CODE BEGIN Header_shoot_task */
/**
* @brief Function implementing the SHOOT thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_shoot_task */
__weak void shoot_task(void const * argument)
{
  /* USER CODE BEGIN shoot_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END shoot_task */
}

/* USER CODE BEGIN Header_push_task */
/**
* @brief Function implementing the PUSH thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_push_task */
__weak void push_task(void const * argument)
{
  /* USER CODE BEGIN push_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END push_task */
}

/* USER CODE BEGIN Header_can2_cmd_task */
/**
* @brief Function implementing the CAN2_CMD thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_can2_cmd_task */
__weak void can2_cmd_task(void const * argument)
{
  /* USER CODE BEGIN can2_cmd_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END can2_cmd_task */
}

/* USER CODE BEGIN Header_pneumatic_task */
/**
* @brief Function implementing the PNEUMATIC thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_pneumatic_task */
__weak void pneumatic_task(void const * argument)
{
  /* USER CODE BEGIN pneumatic_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END pneumatic_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
