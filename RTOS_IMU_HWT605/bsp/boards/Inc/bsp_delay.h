#ifndef BSP_DELAY_H
#define BSP_DELAY_H

#ifdef __cplusplus
 extern "C" {
#endif


#include "struct_typedef.h"

void Delay_Init(uint16_t sysclk);
void delay_us(uint32_t nus);
void delay_ms(uint16_t nms);


#ifdef __cplusplus
}
#endif


#endif

