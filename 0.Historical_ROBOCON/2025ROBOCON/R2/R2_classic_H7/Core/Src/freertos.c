/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
// osThreadId StartLaunchTaskHandle;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId Classic_TaskHandle;
osThreadId Chat_TaskHandle;
osThreadId shout_ballTaskHandle;
osThreadId ChangeAngleTaskHandle;
osThreadId myTask06Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
// void StartLaunchTask(void const * argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void Classic_Move(void const * argument);
void Chat_with_upper(void const * argument);
void Shout_Ball(void const * argument);
void Chang_Shout_Angle(void const * argument);
void Task_Changes(void const * argument);

extern void MX_USB_DEVICE_Init(void);
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

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of Classic_Task */
  osThreadDef(Classic_Task, Classic_Move, osPriorityNormal, 0, 128);
  Classic_TaskHandle = osThreadCreate(osThread(Classic_Task), NULL);

  /* definition and creation of Chat_Task */
  osThreadDef(Chat_Task, Chat_with_upper, osPriorityNormal, 0, 128);
  Chat_TaskHandle = osThreadCreate(osThread(Chat_Task), NULL);

  /* definition and creation of shout_ballTask */
  osThreadDef(shout_ballTask, Shout_Ball, osPriorityNormal, 0, 128);
  shout_ballTaskHandle = osThreadCreate(osThread(shout_ballTask), NULL);

  /* definition and creation of ChangeAngleTask */
  osThreadDef(ChangeAngleTask, Chang_Shout_Angle, osPriorityNormal, 0, 128);
  ChangeAngleTaskHandle = osThreadCreate(osThread(ChangeAngleTask), NULL);

  /* definition and creation of myTask06 */
  osThreadDef(myTask06, Task_Changes, osPriorityNormal, 0, 128);
  myTask06Handle = osThreadCreate(osThread(myTask06), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  // osThreadDef(LaunchTask, StartLaunchTask, osPriorityNormal, 0, 128);  // ×¢²áÈÎÎñ
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
__weak void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Classic_Move */
/**
* @brief Function implementing the Classic_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Classic_Move */
__weak void Classic_Move(void const * argument)
{
  /* USER CODE BEGIN Classic_Move */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Classic_Move */
}

/* USER CODE BEGIN Header_Chat_with_upper */
/**
* @brief Function implementing the Chat_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Chat_with_upper */
__weak void Chat_with_upper(void const * argument)
{
  /* USER CODE BEGIN Chat_with_upper */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Chat_with_upper */
}

/* USER CODE BEGIN Header_Shout_Ball */
/**
* @brief Function implementing the shout_ballTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Shout_Ball */
__weak void Shout_Ball(void const * argument)
{
  /* USER CODE BEGIN Shout_Ball */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Shout_Ball */
}

/* USER CODE BEGIN Header_Chang_Shout_Angle */
/**
* @brief Function implementing the ChangeAngleTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Chang_Shout_Angle */
__weak void Chang_Shout_Angle(void const * argument)
{
  /* USER CODE BEGIN Chang_Shout_Angle */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Chang_Shout_Angle */
}

/* USER CODE BEGIN Header_Task_Changes */
/**
* @brief Function implementing the myTask06 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task_Changes */
__weak void Task_Changes(void const * argument)
{
  /* USER CODE BEGIN Task_Changes */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Task_Changes */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
// __weak void StartLaunchTask(void const * argument)
// {

// }
/* USER CODE END Application */
