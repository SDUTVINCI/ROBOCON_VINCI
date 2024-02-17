#include "Ultrasonic_ranging.h"

/* 输入捕获状态(UR_CAP_STA)
 * [7]  :0,没有成功的捕获;1,成功捕获到一次.
 * [6]  :0,还没捕获到高电平;1,已经捕获到高电平了.
 * [5:0]:捕获高电平后溢出的次数,最多溢出63次,所以最长捕获值 = 63*65536 + 65535 = 4194303
 *       注意:为了通用,我们默认ARR和CCRy都是16位寄存器,对于32位的定时器(如:TIM5),也只按16位使用
 *       按1us的计数频率,最长溢出时间为:4194303 us, 约4.19秒
 *
 *      (说明一下：正常32位定时器来说,1us计数器加1,溢出时间:4294秒)
 */
uint8_t UR_CAP_STA = 0;    /* 输入捕获状态 */
uint16_t UR_CAP_VAL = 0;   /* 输入捕获0值 */

fp32 ur_time;
fp32 ur_distance;
fp32 ur_sound_speed;
fp32 cel_temp = 25.0f;

void UR_Trig_Start(void)
{
	HAL_GPIO_WritePin(Trig_GPIO_Port,Trig_Pin,GPIO_PIN_SET);
	delay_us(10);
	HAL_GPIO_WritePin(Trig_GPIO_Port,Trig_Pin,GPIO_PIN_RESET);
	delay_ms(200);
}



void UR_distance_calc(void)
{
	if (UR_CAP_STA & 0x80)           /* 成功捕获到了一次高电平 */
  {
	 ur_sound_speed = cel_temp * 0.61f + 331.45f;
   ur_time = UR_CAP_STA & 0x3F;
   ur_time *= 0xFFFF;                     /* 溢出时间总和 */
   ur_time += UR_CAP_VAL;          /* 得到总的高电平时间 */
//	 ur_distance = ur_time * ur_sound_speed / 2; /* 打印总的高电平时间 */
	 ur_distance = ur_time/ 58.0f;
   UR_CAP_STA = 0;              /* 开启下一次捕获 */
  }
	
}


/**
 * @brief       定时器输入捕获中断处理回调函数
 * @param       htim:定时器句柄指针
 * @note        该函数在HAL_TIM_IRQHandler中会被调用
 * @retval      无
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM1)
	{
		if ((UR_CAP_STA & 0x80) == 0)    /* 还未成功捕获 */
    {
        if (UR_CAP_STA & 0x40)       /* 捕获到一个下降沿 */
        {
            UR_CAP_STA |= 0x80;      /* 标记成功捕获到一次高电平脉宽 */
            UR_CAP_VAL = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1);  /* 获取当前的捕获值 */
            TIM_RESET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_1);                      /* 一定要先清除原来的设置 */
            TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_1, TIM_ICPOLARITY_RISING); /* 配置TIM1通道1上升沿捕获 */
        }
        else                                            /* 还未开始,第一次捕获上升沿 */
        {
            UR_CAP_STA = 0;                      /* 清空 */
            UR_CAP_VAL = 0;
            UR_CAP_STA |= 0x40;                  /* 标记捕获到了上升沿 */
            __HAL_TIM_DISABLE(&htim1);  /* 关闭定时器1 */
            __HAL_TIM_SET_COUNTER(&htim1, 0);  /* 定时器1计数器清零 */
            TIM_RESET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_1);                       /* 一定要先清除原来的设置！！ */
            TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_1, TIM_ICPOLARITY_FALLING); /* 定时器1通道1设置为下降沿捕获 */
            __HAL_TIM_ENABLE(&htim1);                                                   /* 使能定时器1 */
        }
    }
	}
}


/**
 * @brief       定时器更新中断回调函数
 * @param       htim : 定时器句柄指针
 * @note        此函数会被定时器中断函数共同调用的
 * @retval      无
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM1)
	{
		if ((UR_CAP_STA & 0x80) == 0)                /* 还没成功捕获 */
        {
            if (UR_CAP_STA & 0x40)                   /* 已经捕获到高电平了 */
            {
                if ((UR_CAP_STA & 0x3F) == 0x3F)     /* 高电平太长了 */
                {
                    TIM_RESET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_1);                     /* 一定要先清除原来的设置 */
                    TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_1, TIM_ICPOLARITY_RISING);/* 配置TIM5通道1上升沿捕获 */
                    UR_CAP_STA |= 0x80;              /* 标记成功捕获了一次 */
                    UR_CAP_VAL = 0xFFFF;
                }
                else  /* 累计定时器溢出次数 */
                {
                    UR_CAP_STA++;
                }
            }
        }
	}
}




