#ifndef _data_pack_
#define _data_pack_
typedef enum
{
                Pc = 1,
                Chassis = 2

} Usart_device;
#include "include.h"
void Send_Cmd_Data2chassis(uint8_t cmd, const uint8_t *datas, uint8_t len);
void Send_Cmd_Data2pc(uint8_t cmd, const uint8_t *datas, uint8_t len);
void ReceivefromPc(uint8_t bytedata);
void ReceivefromChassis(uint8_t bytedata);
void SendPakageWaitACK(int Device, uint8_t Cmd, uint8_t *SendData, uint8_t SendDataLen, uint8_t AckData);
void WaitPakageSendACK(int Device, uint8_t Cmd, uint8_t *ReciveData, uint8_t DataLen, uint8_t AckData);
#endif
