#ifndef _data_pack_
#define _data_pack_


#include "include.h"
void Send_Cmd_Data2chassis(uint8_t cmd, const uint8_t *datas, uint8_t len);

void Send_Cmd_Data2Gimbal(uint8_t cmd, const uint8_t *datas, uint8_t len);
void ReceivefromChassis(uint8_t bytedata);
#endif
