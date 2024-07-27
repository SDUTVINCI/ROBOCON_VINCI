#include "L1S.h"
#include <stdio.h>
#include <stdbool.h>
// HAL_UART_Receive_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)

/****************************************************************************************
 * 描   述：L1模组的十六进制协议命令实现
 * 作   者：slh
 * 日   期: 2023.2.14
 *****************************************************************************************/

/***********************************************************
 * 函数名：HEX_Single_Meas_Cmd
 * 参  数：void
 * 返回值：void
 * 描  述：发送单次测量命令
 ************************************************************/
void HEX_Single_Meas_Cmd(UART_HandleTypeDef *huart)
{
	unsigned char cmd[5] = {0XA5, 0X5A, 0X02, 0X00, 0XFD};
	HAL_UART_Transmit(huart, cmd, 5,1000);
}

/***********************************************************
 * 函数名：HEX_Conti_Meas_Cmd
 * 参  数：void
 * 返回值：void
 * 描  述：发送连续测量命令
 ************************************************************/
void HEX_Conti_Meas_Cmd(UART_HandleTypeDef *huart)
{
	unsigned char cmd[5] = {0XA5, 0X5A, 0X03, 0X00, 0XFC};
	HAL_UART_Transmit(huart, cmd, 5,1000);
}

/***********************************************************
 * 函数名：HEX_FastConti_Meas_Cmd
 * 参  数：void
 * 返回值：void
 * 描  述：发送快速连续测量命令
 ************************************************************/
void HEX_FastConti_Meas_Cmd(UART_HandleTypeDef *huart)
{
	unsigned char cmd[5] = {0XA5, 0X5A, 0X04, 0X00, 0XFB};
	HAL_UART_Transmit(huart, cmd, 5,1000);
}

/***********************************************************
 * 函数名：HEX_FastConti_Meas_Cmd
 * 参  数：void
 * 返回值：void
 * 描  述：发送停止测量命令
 ************************************************************/
void HEX_Stop_Meas_Cmd(UART_HandleTypeDef *huart)
{
	unsigned char cmd[5] = {0XA5, 0X5A, 0X05, 0X00, 0XFA};
	HAL_UART_Transmit(huart, cmd, 5,1000);
}

