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
#include "L1S.h"
#include <math.h>

int16_t dx = 0;
int16_t distance = 0;

/*kalman begen*/
#include "kalman.h"
KALMAN_FILTER Kalman_L1sX1, Kalman_L1sX2, Kalman_L1sY1;
/*kalman end*/
extern unsigned int gdistance_X2;
extern uint8_t cmd_myfromActuator;
extern uint8_t rx_datafromActuator[10];
extern float world_vx, world_vy, world_vomega;
extern fp32 yaw;
extern int step_mapan;
extern uint8_t if_l1s_move_point_task_finish;
extern uint8_t if_mapan_move_point_task_finish;
#define Vxy 4000

#define Ytime1 3290
#define Xtime1 2400
#define Ytime2 2100
uint8_t move_mode = 1; // 底盘运动模式 1：Vx Vy  Vomega 控制运动  ；0：遥控器控制 ；2：控制 omega  Vx  Vy
uint8_t color = 0;     // 0:蓝色  1红色
extern uint8_t target_kuang;

extern uint8_t rx_datafromMapan[10];
extern uint8_t cmd_myfromMapan;
uint8_t L1S_move_postion(int x, int y, fp32 omega);
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
int x_port[6] = {1495, 3248, 2498, 1748, 998, 248};
int y_port[7] = {1900, 350, 350, 350, 350, 350};
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
osThreadId move_taskHandle;
osThreadId myTask_LEDHandle;
osThreadId scheduler_taskHandle;
osThreadId jiaozhun_taskHandle;
osThreadId mapanmoveHandle;
osThreadId daokuangtaskHandle;
osThreadId l1smovepointHandle;
osThreadId firstjiaozhunHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
extern void move_chassis_task(void const * argument);
extern void Task_LED(void const * argument);
void Scheduler_Task(void const * argument);
extern void jiaozhuntask(void const * argument);
extern void mapan_move_point_task(void const * argument);
extern void daokuang_task(void const * argument);
extern void l1s_move_point_task(void const * argument);
extern void first_jiaozhun_task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

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

  /* definition and creation of move_task */
  osThreadDef(move_task, move_chassis_task, osPriorityIdle, 0, 128);
  move_taskHandle = osThreadCreate(osThread(move_task), NULL);

  /* definition and creation of myTask_LED */
  osThreadDef(myTask_LED, Task_LED, osPriorityIdle, 0, 128);
  myTask_LEDHandle = osThreadCreate(osThread(myTask_LED), NULL);

  /* definition and creation of scheduler_task */
  osThreadDef(scheduler_task, Scheduler_Task, osPriorityIdle, 0, 128);
  scheduler_taskHandle = osThreadCreate(osThread(scheduler_task), NULL);

  /* definition and creation of jiaozhun_task */
  osThreadDef(jiaozhun_task, jiaozhuntask, osPriorityIdle, 0, 128);
  jiaozhun_taskHandle = osThreadCreate(osThread(jiaozhun_task), NULL);

  /* definition and creation of mapanmove */
  osThreadDef(mapanmove, mapan_move_point_task, osPriorityIdle, 0, 128);
  mapanmoveHandle = osThreadCreate(osThread(mapanmove), NULL);

  /* definition and creation of daokuangtask */
  osThreadDef(daokuangtask, daokuang_task, osPriorityIdle, 0, 128);
  daokuangtaskHandle = osThreadCreate(osThread(daokuangtask), NULL);

  /* definition and creation of l1smovepoint */
  osThreadDef(l1smovepoint, l1s_move_point_task, osPriorityIdle, 0, 128);
  l1smovepointHandle = osThreadCreate(osThread(l1smovepoint), NULL);

  /* definition and creation of firstjiaozhun */
  osThreadDef(firstjiaozhun, first_jiaozhun_task, osPriorityIdle, 0, 128);
  firstjiaozhunHandle = osThreadCreate(osThread(firstjiaozhun), NULL);

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
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  // vTaskSuspend(move_taskHandle);
  vTaskSuspend(myTask_LEDHandle);
  vTaskSuspend(scheduler_taskHandle);
  vTaskSuspend(jiaozhun_taskHandle);
  vTaskSuspend(mapanmoveHandle);
  vTaskSuspend(l1smovepointHandle);
  vTaskSuspend(daokuangtaskHandle);
  vTaskSuspend(firstjiaozhunHandle);

  // InitKalmanFilter(&Kalman_L1sX1);
  // InitKalmanFilter(&Kalman_L1sX2);
  // InitKalmanFilter(&Kalman_L1sY1);
  osDelay(pdMS_TO_TICKS(2000));
  HEX_FastConti_Meas_Cmd(&huart3);
  HEX_FastConti_Meas_Cmd(&huart4);
  HEX_FastConti_Meas_Cmd(&huart5); // 打开激光测距
  Mapan_Init();

  ///////////////////////debug
  // move_mode = 0;
  // vTaskResume(move_taskHandle);
  // vTaskResume(jiaozhun_taskHandle);

    move_mode = 1;
    uint8_t tx_data_buff[1];
    tx_data_buff[0] = 0x11;
    uint8_t flage_setup = 0;
    while (!((cmd_myfromActuator == 0X21) && ((rx_datafromActuator[0] == 0x31) || (rx_datafromActuator[0] == 0x13))))
    {
      osDelay(20); // 底盘启动区域
    }
    flage_setup = rx_datafromActuator[0];
    while (cmd_myfromActuator == 0x21) // 应答上位机消息
    {
      cmd_myfromActuator = 0xff;
      Send_Cmd_Data2Actuator(0x21, tx_data_buff, 1);
      osDelay(20);
    }

    //    flage_setup = 0x31; ////////////////////********************************************debug */

    vTaskResume(move_taskHandle);
    world_vx = 0;
    world_vomega = 0;
    world_vy = 0;
    move_mode=1;
    switch (flage_setup)
    {
    case 0x31: // 初始位置 蓝色

      ////-------------------------------自动到达三区start
      for (int i = 0; i < Vxy; i += 300)
      {
        world_vy = i;
        osDelay(100);
      }
      world_vy = Vxy;

      osDelay(Ytime1);
      for (int i = Vxy; i > 0; i -= 300)
      {
        world_vy = i;
        osDelay(100);
      }
      world_vx = 0;
      world_vy = 0;
      osDelay(100);
      for (int i = 0; i < -Vxy; i -= 200)
      {
        world_vx = i;
        osDelay(300);
      }

      world_vx = -Vxy;
      world_vy = 0;
      osDelay(Xtime1);
	move_mode=2;
      for (int i = -Vxy; i < 0; i += 300)
      {
        world_vx = i;
        osDelay(100);
      }

      world_vx = 0;
      world_vy = 0;
      osDelay(200);
      for (int i = 0; i < Vxy; i += 300)
      {
        world_vy = i;
        osDelay(3);
      }
      world_vy = Vxy;
      osDelay(Ytime2);
      for (int i = Vxy; i > 0; i -= 10)
      {
        world_vy = i;
        osDelay(3);
      }
      world_vx = 0;
      world_vy = 0;
      //   -------------------------------自动到达三区end

      break;
	case 0x13: // 重试位置蓝色
    world_vx = -Vxy;
      world_vy = Vxy;
	osDelay(300);
	world_vx = -Vxy;
      world_vy = 0;
      osDelay(Xtime1-300);
	move_mode=2;
      for (int i = -Vxy; i < 0; i += 300)
      {
        world_vx = i;
        osDelay(100);
      }

      world_vx = 0;
      world_vy = 0;
      osDelay(200);
      for (int i = 0; i < Vxy; i += 300)
      {
        world_vy = i;
        osDelay(3);
      }
      world_vy = Vxy;
      osDelay(Ytime2);
      for (int i = Vxy; i > 0; i -= 10)
      {
        world_vy = i;
        osDelay(3);
      }
      world_vx = 0;
      world_vy = 0;
      break;
	default:
      break;
    }

    tx_data_buff[0] = 0x77;
    while (!((cmd_myfromActuator == 0X21) && (rx_datafromActuator[0] == 0x11)))
    {
      Send_Cmd_Data2Actuator(0x21, tx_data_buff, 1);
      osDelay(5);
    }
    // vTaskResume(firstjiaozhunHandle);
    vTaskResume(scheduler_taskHandle);
    move_mode = 2;
  

  /* Infinite loop */
  for (;;)
  {

    // HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_11);
    if (cmd_myfromActuator == 0x28)
    {
      cmd_myfromActuator = 0xff;
      memcpy(&distance, &rx_datafromActuator[0], 2);
      memcpy(&dx, &rx_datafromActuator[2], 2);
      cmd_myfromActuator = 0xff;
      move_mode = 2;
      world_vx = (float)distance;
      world_vy = (float)dx;
    }

    // if (cmd_myfromActuator == 0x48)
    // {
    //   rx_datafromActuator[0] = 0x06;
    //   vTaskResume();
    // }

    osDelay(10);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Scheduler_Task */
/**
 * @brief Function implementing the scheduler_task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Scheduler_Task */
void Scheduler_Task(void const * argument)
{
  /* USER CODE BEGIN Scheduler_Task */
  /* Infinite loop */
  for (;;)
  {
    uint8_t txbuf = 0;
    if (cmd_myfromActuator == 0x48)
    {
      if (rx_datafromActuator[0] == 0x06) // 位置校准
      {
        txbuf = 0xff;
        while ((cmd_myfromActuator == 0x48) && (rx_datafromActuator[0] == 0x06))
        {
          cmd_myfromActuator = 0xff;
          Send_Cmd_Data2Actuator(0x48, &txbuf, 1);
          osDelay(10);
        }
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, 1);
        target_kuang = 5;
        osDelay(5);
        vTaskResume(firstjiaozhunHandle);
      }
      cmd_myfromActuator = 0xff;
    }
    else if (cmd_myfromActuator == 0x50) // 到框
    {
      step_mapan = 0;
      target_kuang = rx_datafromActuator[0];
      txbuf = 0xff;
      while (cmd_myfromActuator == 0x50)
      {
        cmd_myfromActuator = 0xff;
        Send_Cmd_Data2Actuator(0x50, &txbuf, 1);
        osDelay(10);
      }
      vTaskResume(daokuangtaskHandle);
    }
    // HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, 1);
    osDelay(3);
  }
  /* USER CODE END Scheduler_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
// x 1500    y 2000

uint8_t imu_move_postion(int x, int y, fp32 omega) // imu校准方向
{
  world_vx = 0;
  world_vy = 0;

  world_vomega = -PID_position_chassis_omega1(omega);
}

/* USER CODE END Application */
