#include "delay_user.h"

void nop_delay_us(uint16_t us)
{
 for(; us > 0; us--)
 {
 for(uint8_t i = 10; i > 0; i--)
 {
 __nop();
 __nop();
 __nop();
 __nop();
 __nop();
 __nop();
 __nop();
 __nop();
 __nop();
 __nop();
 __nop();
 __nop();
 __nop();
 __nop();
 __nop();
 }
 }
}


void nop_delay_ms(uint16_t ms)
{
 for(; ms > 0; ms--)
 {
 nop_delay_us(1000);
 }
}
