#include "can.h"
#include "stm32f4xx.h"

static void CAN1_hook(CanRxMsg *rx_message);        //�ֲ���������������CANͨ�����ݡ�
static void CAN2_hook(CanRxMsg *rx_message);        //�ֲ�����
motor_measure_t motor_can1[8];      //�����������ṹ���������������Ƕȣ�ת�ٵȱ���
motor_measure_t motor_can2[8];
CanRxMsg rx1_message;             //�����CANͨ�Ž�����صĽṹ�����������������չID����׼ID��
CanRxMsg rx2_message;
void get_motor_measure(motor_measure_t *ptr,CanRxMsg *rx_message)                           //�ѻ�õ����ݴ�����ܿ�������Ӧ�����ݡ�                           
    {   
//				static int i=0;
//				if(i==0)
//				{
//					ptr->offset_angle = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);
//					i++;
//				}
        (ptr)->last_angle = (ptr)->angle;                                                            
        (ptr)->angle = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);              //����ȡ�Ļ�е�Ƕȣ���8λ�͵�8λ����ƴ�ӡ� 
        (ptr)->speed_rpm = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);          //����ȡ��ת��ת�٣���8λ�͵�8λ����ƴ�ӡ� 
        (ptr)->given_current = (uint16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]);      //����ȡ��ʵ��ת�ص�������8λ�͵�8λ����ƴ�ӡ� 
        (ptr)->temperate = (rx_message)->Data[6];                                              
//				((ptr)->angle) = (int32_t)(((ptr)->ecd) - ((ptr)->last_ecd));

				if(ptr->angle - ptr->last_angle > 4096)           //��¼Ȧ��������ʽ�����������Ǿ���ʽ�����ϵ�ʱ�Ƕ�Ϊ0��
					ptr->round_cnt --;
				else if (ptr->angle - ptr->last_angle < -4096)
					ptr->round_cnt ++;
				ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;         //�ܽǶ� = Ȧ�� * һȦ�ĽǶ�ֵ + ��ǰ�Ƕ� - �����ĽǶ�ֵ��
    }
		
		
void Get_Motor_Offset(motor_measure_t *ptr,CanRxMsg *rx_message)
{
	ptr->angle = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);       //����ȡ�Ļ�е�Ƕȣ���8λ�͵�8λ����ƴ�ӡ� 
	ptr->offset_angle = ptr->angle;                                                    //�����ĽǶ�ֵ = ��ǰ�Ƕ�ֵ��
}


#define ABS(x)	( (x>0) ? (x) : (-x) )
//*@bref ����ϵ�Ƕ�=0�� ֮���������������3510�������Կ�����Ϊ0������ԽǶȡ�


void get_total_angle(motor_measure_t *p)     //����ܽǶ���
{
	int res1, res2, delta;
	if(p->angle < p->last_angle)
		{
			//���ܵ����
		  res1 = p->angle + 8192 - p->last_angle;	//��ת��delta=+
		  res2 = p->angle - p->last_angle;				//��ת	delta=-
	  }else
		{
			//angle > last
		  res1 = p->angle - 8192 - p->last_angle ;//��ת	delta -
		  res2 = p->angle - p->last_angle;				//��ת	delta +
	  }
		//��������ת���϶���ת�ĽǶ�С���Ǹ������
	  if(ABS(res1)<ABS(res2))
			delta = res1;
	  else
		  delta = res2;
		
		p->total_angle += delta;
		p->last_angle = p->angle;
}		



 uint8_t CAN1_mode_init(uint8_t tsjw,uint8_t tbs2,uint8_t tbs1,uint16_t brp,uint8_t mode)     //CANͨ�ų�ʼ��
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
	  

void CAN1_RX0_IRQHandler(void)                     //�жϷ�������ֻҪ����CANͨ�������ݿ��Խ��գ��ʹ����ú������н��ա�
{
    

    if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET)     //�ú������ڼ��CANͨ�Ž����ж��Ƿ���
    {
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);				//���FIFO0��Ϣ�Һ��жϱ�־λ
        CAN_Receive(CAN1, CAN_FIFO0, &rx1_message);     //���������FIFO0�е����ݿ�����rx1_message����ṹ��ĵ�ַ��ȥ��  
		
        CAN1_hook(&rx1_message);                 //�����յ����ݽ��д������е�����ࡣ
			
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


static void CAN1_hook(CanRxMsg *rx_message)  //�����յ����ݽ��д������е�����ࡣ
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
        //������ID
        i = rx_message->StdId - CAN_M1_ID;      //0x201��Ӧ0   0x202��Ӧ1  0x203��Ӧ2
        //���������ݺ꺯��
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
        //������ID
        i = rx_message->StdId - CAN_M1_ID;
        //���������ݺ꺯��
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



