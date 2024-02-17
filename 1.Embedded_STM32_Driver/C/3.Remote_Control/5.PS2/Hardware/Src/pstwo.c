#include "pstwo.h"
/*
  ******************************************************************************
  * @author
  *���紴��ѧ��
	*Vinci�����˶�
	*���ѻ�
  ******************************************************************************
  */


PS2_DataTypedef PS2;
PS2_UserDataTypedef PStwo;


uint8_t Comd[3] = {0x01,0x42,0x00};    //bit3,4�������ֱ�������ֻ����bit0-2,    bit2Ϊ����idle��
uint8_t PS2Data[9] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};


extern SPI_HandleTypeDef hspi1;



void PS2Data_Acquire(void)
{
	PS2Data_Clear();
	PS2Data_Receive();
	PS2Data_Get();
	PS2Rocker_correct();
}
	

void PS2Data_Receive(void)
{
	uint8_t i = 0;
	
	//����Ƭ������ֱ����ݻ����ֱ���������ʱ���������� CS �ߵ�ƽ��
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);   //PA4 SPI1_CS  ��ʼͨ�ŵı�־
	//����һ�����0x01�������տ���idle��
	HAL_SPI_TransmitReceive(&hspi1,&Comd[0],&PS2Data[0],1,0xffff);   
	nop_delay_us(3);
	//��Ƭ�����͡�0x42���������ֱ��������ݣ�����ʱ���յ��ֱ������ġ�0x41�̵�ģʽ����0x73���ģʽ����
	HAL_SPI_TransmitReceive(&hspi1,&Comd[1],&PS2Data[1],1,0xffff);
	nop_delay_us(3);
	//��Ƭ�����Ϳ���idle�����յ��ֱ����͵�0x5a����ʾ���ߵ�Ƭ������Ҫ���ˡ�
	HAL_SPI_TransmitReceive(&hspi1,&Comd[2],&PS2Data[2],1,0xffff);
	nop_delay_us(3); 
	
	for(i = 3;i <9;i++)
	{
	HAL_SPI_TransmitReceive(&hspi1,&Comd[2],&PS2Data[i],1,0xffff); // ��������
	nop_delay_us(3);
	}
	
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);   //PA4 SPI1_CS    ����ͨ�ŵı�־
}



void PS2Data_Get(void)
{  

  
	//Ĭ�������ֲ�Ϊ0���ǰ��£�Ϊ1��û���¡�
	//��λȡ��  ��Ϊ1Ϊ���£�0Ϊû���¡�
	
		PS2Data[3] = ~PS2Data[3];
		PS2Data[4] = ~PS2Data[4];	
	
	
	
	//���ݽ���  
	PS2.SELECT = (PS2Data[3] << 7);
	PS2.L3 = (PS2Data[3] << 6);
	PS2.R3 = (PS2Data[3] << 5);
	PS2.START = (PS2Data[3] << 4);
	PS2.UP = (PS2Data[3] << 3);
	PS2.RIGHT = (PS2Data[3] << 2);
	PS2.DOWN = (PS2Data[3] << 1);
	PS2.LEFT = (PS2Data[3] >> 7);
	
	PS2.L2 = (PS2Data[4] << 7);
	PS2.R2 = (PS2Data[4] << 6);
	PS2.L1 = (PS2Data[4] << 5);
	PS2.R1 = (PS2Data[4] << 4);
	PS2.TRIANGLE = (PS2Data[4] << 3) ;
	PS2.CIRCULAR = (PS2Data[4] << 2);
	PS2.X = (PS2Data[4] << 1);
	PS2.SQUARE = (PS2Data[4] >> 7);
	
	
	
	
	
	
	PS2.SELECT = (PS2.SELECT >> 7);
	PS2.L3 = (PS2.L3 >> 7);
	PS2.R3 = (PS2.R3>> 7);
	PS2.START = (PS2.START >> 7);
	PS2.UP = (PS2.UP>> 7);
	PS2.RIGHT = (PS2.RIGHT >> 7);
	PS2.DOWN = (PS2.DOWN >> 7);
//	PS2.LEFT = (PS2Data[3] >> 0);

	PS2.L2 = (PS2.L2 >> 7);
	PS2.R2 = (PS2.R2>> 7);
	PS2.L1 = (	PS2.L1 >> 7);
	PS2.R1 = (	PS2.R1 >> 7);
	PS2.TRIANGLE = (	PS2.TRIANGLE >> 7);
	PS2.CIRCULAR = (	PS2.CIRCULAR>> 7);
	PS2.X = (	PS2.X >> 7);
//	PS2.SQUARE = (PS2Data[4] >> 0);
	
	
	
	
	
	
//ת��Ϊint��������
	PStwo.SELECT = PS2.SELECT;
	PStwo.L3 = 	PS2.L3;
	PStwo.R3 = PS2.R3;
	PStwo.START = PS2.START;
	PStwo.UP = PS2.UP;
	PStwo.RIGHT = PS2.RIGHT;
	PStwo.DOWN = PS2.DOWN;
	PStwo.LEFT = PS2.LEFT;
	
	PStwo.L2 = PS2.L2;
	PStwo.R2 = PS2.R2;
	PStwo.L1 = PS2.L1;
	PStwo.R1 = PS2.R1;
	PStwo.TRIANGLE = PS2.TRIANGLE;
	PStwo.CIRCULAR = PS2.CIRCULAR;
	PStwo.X = PS2.X;
	PStwo.SQUARE = PS2.SQUARE;

	PS2.PSS_RX = PS2Data[5];   //��0����255
	PS2.PSS_RY = PS2Data[6];	//��0����255
	PS2.PSS_LX = PS2Data[7];
	PS2.PSS_LY = PS2Data[8];
	
	PStwo.PSS_RX = PS2.PSS_RX;
	PStwo.PSS_RY = PS2.PSS_RY;
	PStwo.PSS_LX = PS2.PSS_LX;
	PStwo.PSS_LY = PS2.PSS_LY;
	
}


void PS2Rocker_correct(void)
{
	
	//������ֵ   ��-128  ��0   ��128
	PStwo.PSS_RX -= 128;
	PStwo.PSS_RY -= 127;
	PStwo.PSS_LX -= 128;
	PStwo.PSS_LY -= 127;
	
	//���ٶ�ϵ��
	PStwo.PSS_RX *= 1;
	PStwo.PSS_RY *= -1;
	PStwo.PSS_LX *= 1;
	PStwo.PSS_LY *= -1;
}


void PS2Data_Clear(void)
{
	uint8_t a;
	for(a=0;a<9;a++)
	PS2Data[a]=0x00;
}






