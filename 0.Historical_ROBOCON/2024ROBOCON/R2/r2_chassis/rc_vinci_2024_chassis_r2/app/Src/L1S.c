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
void HEX_Single_Meas_Cmd(UART_HandleTypeDef *huart)
{
	unsigned char cmd[5] = {0XA5, 0X5A, 0X02, 0X00, 0XFD};
	HAL_UART_Transmit(huart, cmd, 5,1000);
}

/***********************************************************
 * ��������HEX_Conti_Meas_Cmd
 * ��  ����void
 * ����ֵ��void
 * ��  ��������������������
 ************************************************************/
void HEX_Conti_Meas_Cmd(UART_HandleTypeDef *huart)
{
	unsigned char cmd[5] = {0XA5, 0X5A, 0X03, 0X00, 0XFC};
	HAL_UART_Transmit(huart, cmd, 5,1000);
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
void HEX_Stop_Meas_Cmd(UART_HandleTypeDef *huart)
{
	unsigned char cmd[5] = {0XA5, 0X5A, 0X05, 0X00, 0XFA};
	HAL_UART_Transmit(huart, cmd, 5,1000);
}

