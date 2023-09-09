#include "can.h"
#include "stm32f4xx.h"

static void CAN1_hook(CanRxMsg *rx_message);        //局部函数，用来接收CAN通信数据。
static void CAN2_hook(CanRxMsg *rx_message);        //局部函数
motor_measure_t motor_can1[8];      //定义了两个结构体变量，用来储存角度，转速等变量
motor_measure_t motor_can2[8];
CanRxMsg rx1_message;             //定义和CAN通信接收相关的结构体变量，用来设置扩展ID，标准ID等
CanRxMsg rx2_message;
void get_motor_measure(motor_measure_t *ptr,CanRxMsg *rx_message)                           //把获得的数据处理成能看懂的相应的数据。                           
    {   
//				static int i=0;
//				if(i==0)
//				{
//					ptr->offset_angle = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);
//					i++;
//				}
        (ptr)->last_angle = (ptr)->angle;                                                            
        (ptr)->angle = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);              //将获取的机械角度，高8位和低8位进行拼接。 
        (ptr)->speed_rpm = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);          //将获取的转子转速，高8位和低8位进行拼接。 
        (ptr)->given_current = (uint16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]);      //将获取的实际转矩电流，高8位和低8位进行拼接。 
        (ptr)->temperate = (rx_message)->Data[6];                                              
//				((ptr)->angle) = (int32_t)(((ptr)->ecd) - ((ptr)->last_ecd));

				if(ptr->angle - ptr->last_angle > 4096)           //记录圈数，增量式编码器（不是绝对式），上电时角度为0。
					ptr->round_cnt --;
				else if (ptr->angle - ptr->last_angle < -4096)
					ptr->round_cnt ++;
				ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;         //总角度 = 圈数 * 一圈的角度值 + 当前角度 - 补偿的角度值。
    }
		
		
void Get_Motor_Offset(motor_measure_t *ptr,CanRxMsg *rx_message)
{
	ptr->angle = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);       //将获取的机械角度，高8位和低8位进行拼接。 
	ptr->offset_angle = ptr->angle;                                                    //补偿的角度值 = 当前角度值。
}


#define ABS(x)	( (x>0) ? (x) : (-x) )
//*@bref 电机上电角度=0， 之后用这个函数更新3510电机的相对开机后（为0）的相对角度。


void get_total_angle(motor_measure_t *p)     //获得总角度数
{
	int res1, res2, delta;
	if(p->angle < p->last_angle)
		{
			//可能的情况
		  res1 = p->angle + 8192 - p->last_angle;	//正转，delta=+
		  res2 = p->angle - p->last_angle;				//反转	delta=-
	  }else
		{
			//angle > last
		  res1 = p->angle - 8192 - p->last_angle ;//反转	delta -
		  res2 = p->angle - p->last_angle;				//正转	delta +
	  }
		//不管正反转，肯定是转的角度小的那个是真的
	  if(ABS(res1)<ABS(res2))
			delta = res1;
	  else
		  delta = res2;
		
		p->total_angle += delta;
		p->last_angle = p->angle;
}		



 uint8_t CAN1_mode_init(uint8_t tsjw,uint8_t tbs2,uint8_t tbs1,uint16_t brp,uint8_t mode)     //CAN通信初始化
{
	GPIO_InitTypeDef GPIO_InitStructure;
	CAN_InitTypeDef CAN_InitStructure;
	CAN_FilterInitTypeDef CAN_FilterInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1,ENABLE);
  RCC_APB1PeriphResetCmd(RCC_APB1Periph_CAN1, ENABLE);
  RCC_APB1PeriphResetCmd(RCC_APB1Periph_CAN1, DISABLE);
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_CAN1);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_CAN1);
	
	  CAN_InitStructure.CAN_TTCM = DISABLE;
    CAN_InitStructure.CAN_ABOM = ENABLE;
    CAN_InitStructure.CAN_AWUM = DISABLE;
    CAN_InitStructure.CAN_NART = ENABLE;
    CAN_InitStructure.CAN_RFLM = DISABLE;
    CAN_InitStructure.CAN_TXFP = DISABLE;
    CAN_InitStructure.CAN_Mode = mode;
    CAN_InitStructure.CAN_SJW = tsjw;
    CAN_InitStructure.CAN_BS1 = tbs1;
    CAN_InitStructure.CAN_BS2 = tbs2;
    CAN_InitStructure.CAN_Prescaler = brp;
    CAN_Init(CAN1, &CAN_InitStructure);
		
		 CAN_FilterInitStructure.CAN_FilterNumber = 0;
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);
  
	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    return 0;

}
uint8_t CAN2_mode_init(uint8_t tsjw,uint8_t tbs2,uint8_t tbs1,uint16_t brp,uint8_t mode)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	CAN_InitTypeDef CAN_InitStructure;
	CAN_FilterInitTypeDef CAN_FilterInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2,ENABLE);
  RCC_APB1PeriphResetCmd(RCC_APB1Periph_CAN2, ENABLE);
  RCC_APB1PeriphResetCmd(RCC_APB1Periph_CAN2, DISABLE);
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_CAN2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2);
	
	  CAN_InitStructure.CAN_TTCM = DISABLE;
    CAN_InitStructure.CAN_ABOM = ENABLE;
    CAN_InitStructure.CAN_AWUM = DISABLE;
    CAN_InitStructure.CAN_NART = ENABLE;
    CAN_InitStructure.CAN_RFLM = DISABLE;
    CAN_InitStructure.CAN_TXFP = DISABLE;
    CAN_InitStructure.CAN_Mode = mode;
    CAN_InitStructure.CAN_SJW = tsjw;
    CAN_InitStructure.CAN_BS1 = tbs1;
    CAN_InitStructure.CAN_BS2 = tbs2;
    CAN_InitStructure.CAN_Prescaler = brp;
    CAN_Init(CAN2, &CAN_InitStructure);
		
		 CAN_FilterInitStructure.CAN_FilterNumber = 14;
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);
  
	CAN_ITConfig(CAN2, CAN_IT_FMP0, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    return 0;

}


void CAN_CMD_can1_1(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    CanTxMsg TxMessage;

    TxMessage.StdId = CAN_CHASSIS_ALL_ID;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = motor1 >> 8;
    TxMessage.Data[1] = motor1;
    TxMessage.Data[2] = motor2 >> 8;
    TxMessage.Data[3] = motor2;
    TxMessage.Data[4] = motor3 >> 8;
    TxMessage.Data[5] = motor3;
    TxMessage.Data[6] = motor4 >> 8;
    TxMessage.Data[7] = motor4;
    
    CAN_Transmit(CHASSIS_CAN, &TxMessage);
	  
}
void CAN_CMD_can1_2(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    CanTxMsg TxMessage;

    TxMessage.StdId = 0x1ff; //0x1ff
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = motor1 >> 8;
    TxMessage.Data[1] = motor1;
    TxMessage.Data[2] = motor2 >> 8;
    TxMessage.Data[3] = motor2;
    TxMessage.Data[4] = motor3 >> 8;
    TxMessage.Data[5] = motor3;
    TxMessage.Data[6] = motor4 >> 8;
    TxMessage.Data[7] = motor4;
    
CAN_Transmit(CHASSIS_CAN, &TxMessage); // can1
	  
}

void CAN_CMD_can2_1(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    CanTxMsg TxMessage;

    TxMessage.StdId = CAN_CHASSIS_ALL_ID;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = motor1 >> 8;
    TxMessage.Data[1] = motor1;
    TxMessage.Data[2] = motor2 >> 8;
    TxMessage.Data[3] = motor2;
    TxMessage.Data[4] = motor3 >> 8;
    TxMessage.Data[5] = motor3;
    TxMessage.Data[6] = motor4 >> 8;
    TxMessage.Data[7] = motor4;
    
   CAN_Transmit(CAN2, &TxMessage);
	  
}
void CAN_CMD_can2_2(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    CanTxMsg TxMessage;

    TxMessage.StdId = 0x1FF;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = motor1 >> 8;
    TxMessage.Data[1] = motor1;
    TxMessage.Data[2] = motor2 >> 8;
    TxMessage.Data[3] = motor2;
    TxMessage.Data[4] = motor3 >> 8;
    TxMessage.Data[5] = motor3;
    TxMessage.Data[6] = motor4 >> 8;
    TxMessage.Data[7] = motor4;
    
CAN_Transmit(CAN2, &TxMessage);
	  
}
	  

void CAN1_RX0_IRQHandler(void)                     //中断服务函数，只要发现CAN通信有数据可以接收，就触发该函数进行接收。
{
    

    if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET)     //该函数用于检查CAN通信接收中断是否发生
    {
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);				//清除FIFO0消息挂号中断标志位
        CAN_Receive(CAN1, CAN_FIFO0, &rx1_message);     //这个函数把FIFO0中的数据拷贝到rx1_message这个结构体的地址中去。  
		
        CAN1_hook(&rx1_message);                 //将接收的数据进行处理，进行电机分类。
			
    }
}

void CAN2_RX0_IRQHandler(void)
{
    

    if (CAN_GetITStatus(CAN2, CAN_IT_FMP0) != RESET)
    {
        CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
        CAN_Receive(CAN2, CAN_FIFO0, &rx2_message);
				
        CAN2_hook(&rx2_message);
			
    }
}


static void CAN1_hook(CanRxMsg *rx_message)  //将接收的数据进行处理，进行电机分类。
{
switch (rx_message->StdId)
{
    case CAN_M1_ID:     //0x201
    case CAN_M2_ID:
    case CAN_M3_ID:
    case CAN_M4_ID:
		case CAN_M5_ID:
		case CAN_M6_ID:
		case CAN_M7_ID:
		case CAN_M8_ID:
						
    {
        static uint8_t i = 0;
        //处理电机ID
        i = rx_message->StdId - CAN_M1_ID;      //0x201对应0   0x202对应1  0x203对应2
        //处理电机数据宏函数
				motor_can1[i].msg_cnt++ <= 50	?	Get_Motor_Offset(&motor_can1[i], rx_message) : get_motor_measure(&motor_can1[i], rx_message);
        get_motor_measure(&motor_can1[i], rx_message);
       
      
        break;
    }
		 default:
    {
        break;
    }
    }

}

static void CAN2_hook(CanRxMsg *rx_message)
{
switch (rx_message->StdId)
{
    case CAN_M1_ID:
    case CAN_M2_ID:
    case CAN_M3_ID:
    case CAN_M4_ID:
		case CAN_M5_ID:
		case CAN_M6_ID:
		case CAN_M7_ID:
		case CAN_M8_ID:
						
    {
        static uint8_t i = 0;
        //处理电机ID
        i = rx_message->StdId - CAN_M1_ID;
        //处理电机数据宏函数
				motor_can2[i].msg_cnt++ <= 50	?	Get_Motor_Offset(&motor_can2[i], rx_message) : get_motor_measure(&motor_can2[i], rx_message);
        get_motor_measure(&motor_can2[i], rx_message);
       
      
        break;
    }
		 default:
    {
        break;
    }
    }

}



