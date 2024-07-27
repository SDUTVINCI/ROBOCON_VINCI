#ifndef _data_pack_
#define _data_pack_
typedef enum
{
                Mapan = 1,
               Actuator = 2

} Usart_device;
#include "include.h"
void ReceivefromMapan(uint8_t bytedata);
void Send_Cmd_Data2Actuator(uint8_t cmd, const uint8_t *datas, uint8_t len);
void Mapan_Init(void);
void Send_Cmd_Data2Mapan(uint8_t cmd, const uint8_t *datas, uint8_t len);
void ReceivefromActuator(uint8_t bytedata);
void SendPakageWaitACK(int Device, uint8_t Cmd, uint8_t *SendData, uint8_t SendDataLen, uint8_t AckData);
void WaitPakageSendACK(int Device, uint8_t Cmd, uint8_t *ReciveData, uint8_t DataLen, uint8_t AckData);

#endif
