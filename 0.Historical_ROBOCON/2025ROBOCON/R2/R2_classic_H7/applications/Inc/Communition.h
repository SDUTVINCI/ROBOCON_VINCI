#ifndef __COMMUNITION_H_
#define __COMMUNITION_H_


#include <cstdint>
#ifdef __cplusplus
extern "C"
{
#endif 

#include "main.h"
#include "usbd_cdc_if.h"
#include "usart.h"


typedef struct 
{
    uint8_t odometer[1];
    uint8_t remote_control[10];
    int16_t remote_sense[4];
    uint8_t upper[1];
        uint8_t DR16[18];
        uint8_t s1,s2;
    uint8_t Buletooth;
    uint8_t MT6701[2];
}Rxbuffer;

uint16_t CRC16_Check(const uint8_t *data, uint8_t len);
void SendByte(uint8_t date);
void Send(const uint8_t *data,uint8_t len);
void Send_Cmd_Data_by_UART(uint8_t cmd,const uint8_t *datas,uint8_t len);
void Data_Analysis_From_upper(uint8_t cmd,uint8_t *datas,uint8_t len);
void Send_Cmd_Data_by_USB(uint8_t cmd,const uint8_t *datas,uint8_t len);



#ifdef __cplusplus
}
#endif 
#endif