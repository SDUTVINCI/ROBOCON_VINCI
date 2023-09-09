#include "Ultrasonic_ranging.h"

/* ���벶��״̬(UR_CAP_STA)
 * [7]  :0,û�гɹ��Ĳ���;1,�ɹ�����һ��.
 * [6]  :0,��û���񵽸ߵ�ƽ;1,�Ѿ����񵽸ߵ�ƽ��.
 * [5:0]:����ߵ�ƽ������Ĵ���,������63��,���������ֵ = 63*65536 + 65535 = 4194303
 *       ע��:Ϊ��ͨ��,����Ĭ��ARR��CCRy����16λ�Ĵ���,����32λ�Ķ�ʱ��(��:TIM5),Ҳֻ��16λʹ��
 *       ��1us�ļ���Ƶ��,����ʱ��Ϊ:4194303 us, Լ4.19��
 *
 *      (˵��һ�£�����32λ��ʱ����˵,1us��������1,���ʱ��:4294��)
 */
uint8_t UR_CAP_STA = 0;    /* ���벶��״̬ */
uint16_t UR_CAP_VAL = 0;   /* ���벶��0ֵ */

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
	if (UR_CAP_STA & 0x80)           /* �ɹ�������һ�θߵ�ƽ */
  {
	 ur_sound_speed = cel_temp * 0.61f + 331.45f;
   ur_time = UR_CAP_STA & 0x3F;
   ur_time *= 0xFFFF;                     /* ���ʱ���ܺ� */
   ur_time += UR_CAP_VAL;          /* �õ��ܵĸߵ�ƽʱ�� */
//	 ur_distance = ur_time * ur_sound_speed / 2; /* ��ӡ�ܵĸߵ�ƽʱ�� */
	 ur_distance = ur_time/ 58.0f;
   UR_CAP_STA = 0;              /* ������һ�β��� */
  }
	
}


/**
 * @brief       ��ʱ�����벶���жϴ���ص�����
 * @param       htim:��ʱ�����ָ��
 * @note        �ú�����HAL_TIM_IRQHandler�лᱻ����
 * @retval      ��
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM1)
	{
		if ((UR_CAP_STA & 0x80) == 0)    /* ��δ�ɹ����� */
    {
        if (UR_CAP_STA & 0x40)       /* ����һ���½��� */
        {
            UR_CAP_STA |= 0x80;      /* ��ǳɹ�����һ�θߵ�ƽ���� */
            UR_CAP_VAL = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1);  /* ��ȡ��ǰ�Ĳ���ֵ */
            TIM_RESET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_1);                      /* һ��Ҫ�����ԭ�������� */
            TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_1, TIM_ICPOLARITY_RISING); /* ����TIM1ͨ��1�����ز��� */
        }
        else                                            /* ��δ��ʼ,��һ�β��������� */
        {
            UR_CAP_STA = 0;                      /* ��� */
            UR_CAP_VAL = 0;
            UR_CAP_STA |= 0x40;                  /* ��ǲ����������� */
            __HAL_TIM_DISABLE(&htim1);  /* �رն�ʱ��1 */
            __HAL_TIM_SET_COUNTER(&htim1, 0);  /* ��ʱ��1���������� */
            TIM_RESET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_1);                       /* һ��Ҫ�����ԭ�������ã��� */
            TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_1, TIM_ICPOLARITY_FALLING); /* ��ʱ��1ͨ��1����Ϊ�½��ز��� */
            __HAL_TIM_ENABLE(&htim1);                                                   /* ʹ�ܶ�ʱ��1 */
        }
    }
	}
}


/**
 * @brief       ��ʱ�������жϻص�����
 * @param       htim : ��ʱ�����ָ��
 * @note        �˺����ᱻ��ʱ���жϺ�����ͬ���õ�
 * @retval      ��
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM1)
	{
		if ((UR_CAP_STA & 0x80) == 0)                /* ��û�ɹ����� */
        {
            if (UR_CAP_STA & 0x40)                   /* �Ѿ����񵽸ߵ�ƽ�� */
            {
                if ((UR_CAP_STA & 0x3F) == 0x3F)     /* �ߵ�ƽ̫���� */
                {
                    TIM_RESET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_1);                     /* һ��Ҫ�����ԭ�������� */
                    TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_1, TIM_ICPOLARITY_RISING);/* ����TIM5ͨ��1�����ز��� */
                    UR_CAP_STA |= 0x80;              /* ��ǳɹ�������һ�� */
                    UR_CAP_VAL = 0xFFFF;
                }
                else  /* �ۼƶ�ʱ��������� */
                {
                    UR_CAP_STA++;
                }
            }
        }
	}
}




