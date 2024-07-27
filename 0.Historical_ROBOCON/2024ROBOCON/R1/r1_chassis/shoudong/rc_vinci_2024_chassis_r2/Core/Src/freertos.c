/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
extern uint8_t result;
#include "L1S.h"
extern uint8_t move_mode_flage;
extern uint8_t nrf_key[20];

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
osThreadId defaultTaskHandle;
osThreadId myTask02Handle;
osThreadId myTask_LEDHandle;
osThreadId myTask04Handle;
osThreadId omega_taskHandle;
osThreadId frist_zoneHandle;
osThreadId myTask07Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const *argument);
extern void StartTask02(void const *argument);
extern void Task_LED(void const *argument);
extern void Task_nrf_reserve(void const *argument);
extern void Pid_omega_task(void const *argument);
extern void frist_zone_task(void const *argument);
void StartTask07(void const *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize);

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
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
void MX_FREERTOS_Init(void)
{
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

  /* definition and creation of myTask02 */
  osThreadDef(myTask02, StartTask02, osPriorityIdle, 0, 128);
  myTask02Handle = osThreadCreate(osThread(myTask02), NULL);

  /* definition and creation of myTask_LED */
  osThreadDef(myTask_LED, Task_LED, osPriorityIdle, 0, 128);
  myTask_LEDHandle = osThreadCreate(osThread(myTask_LED), NULL);

  /* definition and creation of myTask04 */
  osThreadDef(myTask04, Task_nrf_reserve, osPriorityIdle, 0, 128);
  myTask04Handle = osThreadCreate(osThread(myTask04), NULL);

  /* definition and creation of omega_task */
  osThreadDef(omega_task, Pid_omega_task, osPriorityIdle, 0, 128);
  omega_taskHandle = osThreadCreate(osThread(omega_task), NULL);

  /* definition and creation of frist_zone */
  osThreadDef(frist_zone, frist_zone_task, osPriorityIdle, 0, 128);
  frist_zoneHandle = osThreadCreate(osThread(frist_zone), NULL);

  /* definition and creation of myTask07 */
  osThreadDef(myTask07, StartTask07, osPriorityIdle, 0, 128);
  myTask07Handle = osThreadCreate(osThread(myTask07), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  vTaskSuspend(omega_taskHandle);
  //vTaskSuspend(myTask02Handle);
  vTaskSuspend(frist_zoneHandle);

  osDelay(2400);
  HEX_FastConti_Meas_Cmd(&huart5);
  HEX_FastConti_Meas_Cmd(&huart1);
  HEX_FastConti_Meas_Cmd(&huart3);
  HEX_FastConti_Meas_Cmd(&huart6);
  /* Infinite loop */
  for (;;)
  {

 
       

      osDelay(10);
      //	vTaskResume(right_qvmiaoHandle);
    

    osDelay(20);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask07 */
/**
 * @brief Function implementing the myTask07 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask07 */
void StartTask07(void const *argument)
{
  /* USER CODE BEGIN StartTask07 */
  /* Infinite loop */
  for (;;)
  {

    osDelay(10);
  }

  /* USER CODE END StartTask07 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
