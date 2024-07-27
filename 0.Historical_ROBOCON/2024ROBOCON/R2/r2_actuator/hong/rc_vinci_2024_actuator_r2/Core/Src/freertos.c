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
#include "data_pack.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern uint8_t cmd_myfromPc;
extern uint8_t rx_datafromPc[10];
extern uint8_t cmd_myfromChassis;
extern uint8_t rx_datafromChassis[10];
extern int motor_speed[3];

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
osThreadId motor_taskHandle;
osThreadId track_ballHandle;
osThreadId scheduler_TaskHandle;
osThreadId zhaokaungtaskHandle;
osThreadId jiaozhuntaskHandle;
osThreadId grapballtaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const *argument);
extern void motor_control_task(void const *argument);
extern void track_ball_task(void const *argument);
extern void scheduler_task(void const *argument);
extern void zhaokaung_task(void const *argument);
extern void jiaozhun_task(void const *argument);
extern void grapball_task(void const *argument);

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

  /* definition and creation of motor_task */
  osThreadDef(motor_task, motor_control_task, osPriorityNormal, 0, 128);
  motor_taskHandle = osThreadCreate(osThread(motor_task), NULL);

  /* definition and creation of track_ball */
  osThreadDef(track_ball, track_ball_task, osPriorityIdle, 0, 128);
  track_ballHandle = osThreadCreate(osThread(track_ball), NULL);

  /* definition and creation of scheduler_Task */
  osThreadDef(scheduler_Task, scheduler_task, osPriorityHigh, 0, 128);
  scheduler_TaskHandle = osThreadCreate(osThread(scheduler_Task), NULL);

  /* definition and creation of zhaokaungtask */
  osThreadDef(zhaokaungtask, zhaokaung_task, osPriorityIdle, 0, 128);
  zhaokaungtaskHandle = osThreadCreate(osThread(zhaokaungtask), NULL);

  /* definition and creation of jiaozhuntask */
  osThreadDef(jiaozhuntask, jiaozhun_task, osPriorityIdle, 0, 128);
  jiaozhuntaskHandle = osThreadCreate(osThread(jiaozhuntask), NULL);

  /* definition and creation of grapballtask */
  osThreadDef(grapballtask, grapball_task, osPriorityIdle, 0, 128);
  grapballtaskHandle = osThreadCreate(osThread(grapballtask), NULL);

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

  vTaskSuspend(scheduler_TaskHandle);
  vTaskSuspend(grapballtaskHandle);
  vTaskSuspend(jiaozhuntaskHandle);
  vTaskSuspend(track_ballHandle);
  vTaskSuspend(zhaokaungtaskHandle);
  osDelay(1000);
  uint8_t TX_BUFFER = 0;
  /******************************************************************** */
  int8_t flog_setup = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3); // 判断球的颜色

  switch (flog_setup)
  {
  case 0: // 红色
    TX_BUFFER = 0X22;
    break;
  case 1: // 蓝色
    TX_BUFFER = 0X11;
    break;
  default:
    break;
  }

  while (!((cmd_myfromPc == 0X00) && (rx_datafromPc[0] == 0x01))) // 复位与PC校验
  {
    Send_Cmd_Data2pc(0x00, &TX_BUFFER, 1);
    osDelay(10);
  }

  /***************************************************************************** */
  flog_setup = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4); // 判断启动位置
  switch (flog_setup)
  {
  case 0: // 初始位置
    TX_BUFFER = 0X31;
    break;
  case 1: // 重试区
    TX_BUFFER = 0X13;
    break;
  default:
    break;
  }
  while (!((cmd_myfromChassis == 0X21) && (rx_datafromChassis[0] == 0x11))) // 控制底盘 在启动区启动还是在重试区启动
  {
    Send_Cmd_Data2chassis(0x21, &TX_BUFFER, 1);
    osDelay(15);
  }
  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_11);
  while (!((cmd_myfromChassis == 0X21) && (rx_datafromChassis[0] == 0x77)))
  {
    osDelay(10);
  }
  TX_BUFFER = 0x11;
  while ((cmd_myfromChassis == 0x21) && (rx_datafromChassis[0] == 0x77))
  {
    cmd_myfromChassis = 0xff;
    Send_Cmd_Data2chassis(0x21, &TX_BUFFER, 1);
    osDelay(5);
  }

  // TX_BUFFER = 0X01;
  // SendPakageWaitACK(Pc, 0x01, &TX_BUFFER, 1, 0x01);
  // 向pc发送0x01信息已经到达
  //  while (!((cmd_myfromPc == 0X01) && (rx_datafromPc[0] == 0x01))) //
  //  {
  //    Send_Cmd_Data2pc(0x01, &TX_BUFFER, 1);
  //    osDelay(10);
  //  }

  vTaskResume(scheduler_TaskHandle);

  /* Infinite loop */
  for (;;)
  {

    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_11);
    osDelay(600);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */ 

/* USER CODE END Application */
