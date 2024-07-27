#ifndef _l1s_h_
#define _l1s_h_

#include "include.h"
void L1S_Read_Distance(uint8_t i, uint8_t ch, int *distance);
void HEX_FastConti_Meas_Cmd(UART_HandleTypeDef *huart);
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
void HEX_Single_Meas_Cmd(void);
/***********************************************************
 * ��������HEX_Conti_Meas_Cmd
 * ��  ����void
 * ����ֵ��void
 * ��  ��������������������
 ************************************************************/
void HEX_Conti_Meas_Cmd(void);

/***********************************************************
 * ��������HEX_FastConti_Meas_Cmd
 * ��  ����void
 * ����ֵ��void
 * ��  �������Ϳ���������������
 ************************************************************/
//void HEX_FastConti_Meas_Cmd(UART_HandleTypeDef *huart);
/***********************************************************
 * ��������HEX_FastConti_Meas_Cmd
 * ��  ����void
 * ����ֵ��void
 * ��  ��������ֹͣ��������
 ************************************************************/
void HEX_Stop_Meas_Cmd(void);
#endif
