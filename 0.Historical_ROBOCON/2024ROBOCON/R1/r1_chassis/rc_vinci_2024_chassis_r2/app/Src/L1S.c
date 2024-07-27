#include "L1S.h"
#include <stdio.h>
#include <stdbool.h>
// HAL_UART_Receive_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)

/****************************************************************************************
 * ��   ����L1ģ���ʮ������Э������ʵ��
 * ��   �ߣ�slh
 * ��   ��: 2023.2.14
 *****************************************************************************************/

/***********************************************************
 * ��������HEX_Single_Meas_Cmd
 * ��  ����void
 * ����ֵ��void
 * ��  �������͵��β�������
 ************************************************************/
void HEX_Single_Meas_Cmd(void)
{
	unsigned char cmd[5] = {0XA5, 0X5A, 0X02, 0X00, 0XFD};
	HAL_UART_Transmit(&huart6, cmd, 5,1000);
}

/***********************************************************
 * ��������HEX_Conti_Meas_Cmd
 * ��  ����void
 * ����ֵ��void
 * ��  ��������������������
 ************************************************************/
void HEX_Conti_Meas_Cmd(void)
{
	unsigned char cmd[5] = {0XA5, 0X5A, 0X03, 0X00, 0XFC};
	HAL_UART_Transmit(&huart6, cmd, 5,1000);
}

/***********************************************************
 * ��������HEX_FastConti_Meas_Cmd
 * ��  ����void
 * ����ֵ��void
 * ��  �������Ϳ���������������
 ************************************************************/
void HEX_FastConti_Meas_Cmd(UART_HandleTypeDef *huart)
{
	unsigned char cmd[5] = {0XA5, 0X5A, 0X04, 0X00, 0XFB};
	HAL_UART_Transmit(huart, cmd, 5,1000);
}

/***********************************************************
 * ��������HEX_FastConti_Meas_Cmd
 * ��  ����void
 * ����ֵ��void
 * ��  ��������ֹͣ��������
 ************************************************************/
void HEX_Stop_Meas_Cmd(void)
{
	unsigned char cmd[5] = {0XA5, 0X5A, 0X05, 0X00, 0XFA};
	HAL_UART_Transmit(&huart6, cmd, 5,1000);
}
static unsigned int gresult[2] = {0};
static bool fstatus[2] = {false, false};
static unsigned int gdistance[2] = {0};
static int gbytes[2] = {0};

void L1S_Read_Distance(uint8_t i, uint8_t ch, int *distance)
{

	if (!fstatus[i - 1])
	{
		gresult[i - 1] = gresult[i - 1] | ch;
		if ((gresult[i - 1] & 0xffffff) == 0XB46903) // ֡ͷ��B4 69 03
		{
			fstatus[i - 1] = true; // �ҵ�֡ͷ��
			gresult[i - 1] = 0;
		}
		else
		{
			gresult[i - 1] = gresult[i - 1] << 8;
		}
	}
	else
	{
		gbytes[i - 1]++;
		gdistance[i - 1] = gdistance[i - 1] | ch;
		if (gbytes[i - 1] != 4)
		{
			gdistance[i - 1] = gdistance[i - 1] << 8;
		}
		else
		{
			*distance = gdistance[i - 1];
			gbytes[i - 1] = 0;
			gdistance[i - 1] = 0;
			fstatus[i - 1] = false;
		}
	}
}
