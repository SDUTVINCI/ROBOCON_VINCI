#include "Communition.h"
#include "bsp_usart.h"
#include "chassic.h"
#include "usbd_cdc.h"


Rxbuffer all_rxbuff;


uint16_t CRC16_Check(const uint8_t *data, uint8_t len)
{
	uint16_t CRC16 = 0xFFFF;
	uint8_t state, i, j;
	for (i = 0; i < len; i++)
	{
		CRC16 ^= data[i];
		for (j = 0; j < 8; j++)
		{
			state = CRC16 & 0x01;
			CRC16 >>= 1;
			if (state)
			{
				CRC16 ^= 0xA001;
			}
		}
	}
	return CRC16;
}
void SendByte(uint8_t date)
{
        uint8_t buffer[1];
        buffer[0] = date;  
	HAL_UART_Transmit( &huart1 , buffer , 1,100 );
}

void Send(const uint8_t *data,uint8_t len)
{
        uint8_t i;
        for (i = 0; i < len; i++)
        {
            SendByte(data[i]);//发送一个字节
        }
}


//发送数据包
void Send_Cmd_Data_by_UART(uint8_t cmd,const uint8_t *datas,uint8_t len)
{
    uint8_t buf[300],i,cnt=0;
    uint16_t crc16;
    buf[cnt++] = 0xA5;
    buf[cnt++] = 0x5A;
    buf[cnt++] = len;
    buf[cnt++] = cmd;
    for(i=0;i<len;i++)
    {
        buf[cnt++] = datas[i];
    }
    crc16 = CRC16_Check(buf,len+4);
    buf[cnt++] = crc16>>8;
    buf[cnt++] = crc16&0xFF;
    buf[cnt++] = 0xFF;
	HAL_UART_Transmit( &huart1 , buf , cnt,100 );
    Send(buf,cnt);//调用数据帧发送函数将打包好的数据帧发送出去
}


void Send_Cmd_Data_by_USB(uint8_t cmd,const uint8_t *datas,uint8_t len)
{
    uint8_t buf[300],i,cnt=0;
    uint16_t crc16;
    buf[cnt++] = 0xA5;
    buf[cnt++] = 0x5A;
    buf[cnt++] = len;
    buf[cnt++] = cmd;
    for(i=0;i<len;i++)
    {
        buf[cnt++] = datas[i];
    }
    crc16 = CRC16_Check(buf,len+4);
    buf[cnt++] = crc16>>8;
    buf[cnt++] = crc16&0xFF;
    buf[cnt++] = 0xFF;
    CDC_Transmit_HS(buf,cnt);
}

//数据帧解析
void Data_Analysis_From_upper(uint8_t cmd,uint8_t *datas,uint8_t len)
{
    /*navagition test data*/
//            memcpy(&robot.chassic.Chassic_Move_Data.vx,&datas[0],4);
//            memcpy(&robot.chassic.Chassic_Move_Data.vy,&datas[4],4);
//            memcpy(&robot.chassic.Chassic_Move_Data.vomega,&datas[8],4); 
            /*navagition test data*/
        robot.robot_commend = cmd;
        
        memcpy(&robot.upper_data[0],&datas[0],4);//X
        memcpy(&robot.upper_data[1],&datas[4],4);//Y
        memcpy(&robot.upper_data[2],&datas[8],4); //OMEGA
        memcpy(&robot.upper_data[3],&datas[12],4); //距离
        memcpy(&robot.upper_data[4],&datas[16],4); //SPEED_rpm


}

//接收数据
extern "C"
void Receive_From_upper(uint8_t bytedata)
{

        static uint8_t step=0;//状态变量初始化为0 在函数中必须为静态变量
        static uint8_t cnt=0,Buf[300],len,cmd,*data_ptr;
        static uint16_t crc16;
        //进行数据解析 状态机
        switch(step)
        {
            case 0://接收帧头1状态
                if(bytedata== 0xA5)
                {
                    step++;
                    cnt = 0;
                    Buf[cnt++] = bytedata;
                }break;
            case 1://接收帧头2状态
                if(bytedata== 0x5A)
                {
                    step++;
                    Buf[cnt++] = bytedata;
                }
                else if(bytedata== 0xA5)
                {
                    step = 1;
                }
                else
                {
                    step = 0;
                }
                break;
            case 2://接收数据长度字节状态
                step++;
                Buf[cnt++] = bytedata;
                len = bytedata;
                break;
            case 3://接收命令字节状态
                step++;
                Buf[cnt++] = bytedata;
                cmd = bytedata;
                data_ptr = &Buf[cnt];//记录数据指针首地址
                if(len == 0)step++;//数据字节长度为0则跳过数据接收状态
                break;
            case 4://接收len字节数据状态
                Buf[cnt++] = bytedata;
                if(data_ptr + len == &Buf[cnt])//利用指针地址偏移判断是否接收完len位数据
                {
                    step++;
                }
                break;
            case 5://接收crc16校验高8位字节
                step++;
                crc16 = bytedata;
                break;
            case 6://接收crc16校验低8位字节
                crc16 <<= 8;
                crc16 += bytedata;
                if(crc16 == CRC16_Check(Buf,cnt))//校验正确进入下一状态
                {
                    step ++;
                }
                else if(bytedata == 0xA5)
                {
                    step = 1;
                }
                else
                {
                    step = 0;
                }
                break;
            case 7://接收帧尾
                if(bytedata== 0xFF)//帧尾接收正确
                {
                        Data_Analysis_From_upper(cmd,data_ptr,len);//数据解析
                    step = 0;
                }
                else if(bytedata == 0xA5)
                {
                    step = 1;
                }
                else
                {
                    step = 0;
                }
                break;
            default:step=0;break;//多余状态，正常情况下不可能出现
        }
}