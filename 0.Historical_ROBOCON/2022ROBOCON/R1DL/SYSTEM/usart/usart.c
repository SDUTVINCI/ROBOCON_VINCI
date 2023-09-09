#include "sys.h"
#include "usart.h"	
#include "string.h"
#include "math.h"

uint8_t sbus_rx_buffer[2][18u];
struct 
 {
 int16_t ch0;
 int16_t ch1;
 int16_t ch2;
 int16_t ch3;
 int16_t ch4; 
 int8_t s1;
 int8_t s2;
 }rc;



int htoi(char hex);
void RemoteDataProcess(uint8_t *pData);
void CopeSerial2Data(unsigned char ucData);
 
 /*--------------------------------------------------------------------------------------*/
//USART3----遥控器串口
void uart_init(u32 bound){
   //GPIO端口设置
  GPIO_InitTypeDef   GPIO_InitStructure;
	USART_InitTypeDef  USART_InitStructure;
	DMA_InitTypeDef    dma;
	NVIC_InitTypeDef   NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); //使能GPIOC时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);//使能USART3时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);
	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_USART3);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_10; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //上拉
	GPIO_Init(GPIOC,&GPIO_InitStructure); //初始化PA9，PA10

	USART_InitStructure.USART_BaudRate = 100000;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_Even;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx ;	//收发模式
  USART_Init(USART3, &USART_InitStructure); //初始化串口1
	
  USART_Cmd(USART3, ENABLE);  //使能串口1 
	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE); //usart rx idle interrupt enabled
  USART_ITConfig(USART3, USART_IT_ERR, ENABLE);
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
  NVIC_InitStructure.NVIC_IRQChannel =  USART3_IRQn;//串口1中断通道USART2_IRQn
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x02;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0x00;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
	
	 DMA_DeInit(DMA1_Stream1);
	 dma.DMA_Channel = DMA_Channel_4;
	 dma.DMA_PeripheralBaseAddr = (uint32_t)&(USART3->DR);
	 dma.DMA_Memory0BaseAddr = (uint32_t)&sbus_rx_buffer[0][0];
	 dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
	 dma.DMA_BufferSize = 18u;
	 dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	 dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
	 dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	 dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	 dma.DMA_Mode = DMA_Mode_Circular;
	 dma.DMA_Priority = DMA_Priority_VeryHigh;
	 dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
	 dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	 dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	 dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
		DMA_DoubleBufferModeConfig(DMA1_Stream1,(uint32_t)&sbus_rx_buffer[1][0],DMA_Memory_0); 
	//first used memory configuration
	 DMA_DoubleBufferModeCmd(DMA1_Stream1, ENABLE);
	 DMA_Init(DMA1_Stream1,&dma);
 
		USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE);
		DMA_Cmd(DMA1_Stream1,ENABLE);
}
////*********************************************************///陀螺仪
struct SAngle 	stcAngle;
float yaw;
float pos_x=0;
float pos_y=0;
float xangle=0;
float yangle=0;
float zangle=0;
float w_z=0;
void USART1_Init(u32 bound)	
{
   //GPIO端口设置
   GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	NVIC_InitTypeDef NVIC_InitStructure;
	


	//usart1
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使能USART1时钟
 
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_USART1); //GPIOA10复用为USART1
	
	//USART1端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA9，PA10
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOB,&GPIO_InitStructure); //初始化PA9，PA10

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART1, &USART_InitStructure); //初始化串口1
	
  USART_Cmd(USART1, ENABLE);  //使能串口1 
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启相关中断

	//Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x02;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0x01;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
	
	//*************************************码盘usart6配置***************************************
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE); //使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);//使能USART1时钟
 
	//串口6对应引脚复用映射
	GPIO_PinAFConfig(GPIOG,GPIO_PinSource14,GPIO_AF_USART6); //GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOG,GPIO_PinSource9,GPIO_AF_USART6); //GPIOA10复用为USART1
	
	//USART6端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_9; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOG,&GPIO_InitStructure); //初始化PA9，PA10

   //USART6 初始化设置
	USART_InitStructure.USART_BaudRate = 115200;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART6, &USART_InitStructure); //初始化串口1
	
  USART_Cmd(USART6, ENABLE);  //使能串口1 
	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);//开启相关中断

	//Usart6 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x02;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0x02;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
}

float ccc = 1.0f;
char 	make_zero[5] = {0xff, 0xaa, 0x76, 0x00, 0x00};
char unlocking[5] = {0xff, 0xaa, 0x69, 0x88, 0xb5};
void Yaw_Init(void)
{
	  USART1_Init(9600);
		for(int t=0;t<5;t++)
	{
		USART_SendData(USART1, unlocking[t]);         
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);
	}
	
//陀螺仪归零
		for(int t=0;t<5;t++)
	{
		USART_SendData(USART1, make_zero[t]);         
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);
	}
	
}


void CopeSerial2Data(unsigned char ucData)
{
	static unsigned char ucRxBuffer[250];
	static unsigned char ucRxCnt = 0;	
	
	

	ucRxBuffer[ucRxCnt++]=ucData;	//将收到的数据存入缓冲区中
	if (ucRxBuffer[0]!=0x55) //数据头不对，则重新开始寻找0x55数据头
	{
		ucRxCnt=0;
		return;
	}
	if (ucRxCnt<11) {return;}//数据不满11个，则返回
	else
	{
		if(ucRxBuffer[1]==0x53)//判断数据是哪种数据，然后将其拷贝到对应的结构体中，有些数据包需要通过上位机打开对应的输出后，才能接收到这个数据包的数据
		{
			memcpy(&stcAngle,&ucRxBuffer[2],8);
		}
		ucRxCnt=0;//清空缓存区
	}
}


void stract(char str1[],char str2[],int num)
{
	int i= 0,j = 0;
	while(str1[i] != '\0') i++;
	for(j=0;j<num;j++)
		str1[i++] = str2[j];
}

void Update_yaw(float new_yaw)
{
//	int i=0;
	char update_yaw[8] = "ACTJ";
	static union
	{
		float yaw;
		char data[4];
	}new_set;
	new_set.yaw = new_yaw;
	stract(update_yaw,new_set.data,4);
//	for(i=0;i<8;i++)
//	{
//		while(USART_GetFlagStatus(UART4,USART_FLAG_TXE)==RESET);
//		USART_SendData(UART4,update_yaw[i]);	
//	}
}


void USART1_IRQHandler(void)
{
	static u8 Res;
	static int qwe=0;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		qwe++;
		Res =USART_ReceiveData(USART1);//(USART1->DR);	//读取接收到的数据
		CopeSerial2Data(Res);
    yaw = (float)stcAngle.Angle[2]/32768*180.0f;
		
  } 
	
}
////****************************底*****************************///陀螺仪




uint8_t ch;

float re_x;
float re_y;
float ops_angle_err;

int aa = 0;

//遥控器
void USART3_IRQHandler(void)
{
  if (USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)
  {
        (void)USART3->SR;
        (void)USART3->DR;

		if(DMA_GetCurrentMemoryTarget(DMA1_Stream1) == 0)
  {
		
   DMA_Cmd(DMA1_Stream1, DISABLE);
   DMA1_Stream1->NDTR = (uint16_t)18u; //relocate the dma memory pointer to the beginning position
   DMA1_Stream1->CR |= (uint32_t)(DMA_SxCR_CT);
//enable the current selected memory is Memory 1

   DMA_Cmd(DMA1_Stream1, ENABLE);
 if(DMA_GetCurrDataCounter(DMA1_Stream1) == 0) //ensure received complete frame data.

  {
	RemoteDataProcess(sbus_rx_buffer[0]);
  }
else 
{
   DMA_Cmd(DMA1_Stream1, DISABLE);
   DMA1_Stream1->NDTR = (uint16_t)18u; //relocate the dma memory pointer to the beginning position

   DMA1_Stream1->CR &= ~(uint32_t)(DMA_SxCR_CT);//enable the current selected memory is Memory 0

   DMA_Cmd(DMA1_Stream1, ENABLE);
 if(DMA_GetCurrDataCounter(DMA1_Stream1)!= 0)
{
RemoteDataProcess(sbus_rx_buffer[1]);
	     }
    }
  }
}
	USART_ClearITPendingBit(USART3,USART_IT_IDLE); 
}

 void RemoteDataProcess(uint8_t *pData)
{

  if (pData == NULL )
    {
        return;
    }

 rc.ch0 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF; 
 rc.ch1 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) 
& 0x07FF;
rc.ch2 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) |
 ((int16_t)pData[4] << 10)) & 0x07FF;
 rc.ch3 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) & 
0x07FF;
 rc.ch4 = (int16_t)pData[16] | ((int16_t)pData[17] << 8);                 //NULL

 rc.s1 = ((pData[5] >> 4) & 0x000C) >> 2;
 rc.s2 = ((pData[5] >> 4) & 0x0003);
	rc.ch0 -= 1024;
  rc.ch1 -= 1024;
  rc.ch2 -= 1024;
  rc.ch3 -= 1024;
  rc.ch4 -= 1024;
}

float len_back,len_right;
float len_back_last,len_right_last;
float LPF_XS = 0.4f;

u8 USART6_RxBuffer[20] = {0};
u8 USART3_RxBuffer[20] = {0};

float PI = 3.14159f;
float ac;
float ab;

int htoi(char hex)
{
	int n = 0;
	if (hex >= 0x30 && hex <= 0x39)
	{
		n = hex - 0x30;
	}
	else if (hex >= 0x41 && hex <= 0x46) 
	{
		n = hex - 0x41 + 10; 
	} 

	return n;
}


//USART6中断服务函数

int Aaa = 0;
uint8_t ch;

void USART6_IRQHandler(void)
{
	Aaa = 1;
	//static uint8_t ch;
	
	static union
	{
		u8 data[24];
		float ActVal[6];
	}posture;
	static uint8_t count=0;
	static uint8_t i=0;
	if(USART_GetITStatus(USART6,USART_IT_RXNE)==SET)
	{
		USART_ClearITPendingBit(USART6,USART_IT_RXNE);
		ch=USART_ReceiveData(USART6);
		switch(count)
		{
			case 0:
			if(ch==0x0d)
				count++;
			else
				count=0;
			break;
			case 1:
			if(ch==0x0a)
			{ 
				i=0;
				count++;
			}
			else if(ch==0x0d);
			else 
				count=0;
			break;
			case 2:
				posture.data[i]=ch;
				i++;
			if(i>=24)
			{
				i=0;
				count++;
			}
			break;
			case 3:
				if(ch==0x0a)
			count++;
			else
				count=0;
			break;
			case 4:
				if(ch==0x0d)
			{
				zangle=posture.ActVal[0];
				xangle=posture.ActVal[1];
				yangle=posture.ActVal[2];
				pos_x=-posture.ActVal[3];
				pos_y=-posture.ActVal[4];
				w_z=posture.ActVal[5];
				
//				ops_angle_err = yaw - zangle;
//				re_x = pos_x*cos(ops_angle_err*0.0174f)-pos_y*sin(ops_angle_err*0.0174f);
//				re_y = pos_y*cos(ops_angle_err*0.0174f)+pos_x*sin(ops_angle_err*0.0174f);
				
			}
			count=0;
			break;
			default:
				count=0;
			break;
			
		}
	}
}





