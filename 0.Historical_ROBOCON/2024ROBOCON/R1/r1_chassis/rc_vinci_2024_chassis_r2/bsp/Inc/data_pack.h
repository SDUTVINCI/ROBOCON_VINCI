#ifndef _data_pack_
#define _data_pack_

#include "include.h"
void Send_Cmd_Data2Actuator(uint8_t cmd, const uint8_t *datas, uint8_t len);
void ReceivefromActuator(uint8_t bytedata);
void Send_Cmd_Data2Mapan(uint8_t cmd, const uint8_t *datas, uint8_t len);
void ReceivefromMapan(uint8_t bytedata);
#endif
