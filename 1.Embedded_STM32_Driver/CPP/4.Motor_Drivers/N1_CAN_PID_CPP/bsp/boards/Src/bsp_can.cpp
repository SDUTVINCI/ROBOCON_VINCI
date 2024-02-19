#include "bsp_can.h"

/**
 * @brief       CAN通信开启函数
 * @param       hcan：CAN通信句柄
 * @retval      void
 * @note        建议先初始化CAN通信滤波函数，再开启CAN通信
 */
void CAN_BUS::BSP::CAN_Start(CAN_HandleTypeDef *hcan)
{
	if(HAL_CAN_Start(hcan) != HAL_OK)
	{
		Error_Handler();
	}

	if(HAL_CAN_ActivateNotification(hcan,CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
	}
}

/**
 * @brief       CAN通信滤波函数
 * @param       hcan：CAN通信句柄
 * @retval      void
 * @note        开启CAN通信接收时，必须初始化该函数，建议将该函数在CAN通信开启前就被初始化
 */
void CAN_BUS::BSP::Filter_Init(CAN_HandleTypeDef *hcan)
{
	if(hcan->Instance == CAN1)
	{
		CAN_FilterTypeDef CAN_Filter_st;
    CAN_Filter_st.FilterActivation = ENABLE;
    CAN_Filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    CAN_Filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    CAN_Filter_st.FilterIdHigh = 0x0000;
    CAN_Filter_st.FilterIdLow = 0x0000;
    CAN_Filter_st.FilterMaskIdHigh = 0x0000;
    CAN_Filter_st.FilterMaskIdLow = 0x0000;
    CAN_Filter_st.FilterBank = 0;
    CAN_Filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
	  if(HAL_CAN_ConfigFilter(hcan, &CAN_Filter_st)!=HAL_OK)       //滤波器初始化
		{
			Error_Handler();
		}
	}
	
	else if(hcan->Instance == CAN2)
	{
		CAN_FilterTypeDef CAN_Filter_st;
    CAN_Filter_st.FilterActivation = ENABLE;
    CAN_Filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    CAN_Filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    CAN_Filter_st.FilterIdHigh = 0x0000;
    CAN_Filter_st.FilterIdLow = 0x0000;
    CAN_Filter_st.FilterMaskIdHigh = 0x0000;
    CAN_Filter_st.FilterMaskIdLow = 0x0000;
    CAN_Filter_st.FilterBank = 14;
		CAN_Filter_st.SlaveStartFilterBank = 14;
    CAN_Filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
	  if(HAL_CAN_ConfigFilter(hcan, &CAN_Filter_st)!=HAL_OK)        //滤波器初始化
		{
			Error_Handler();
		}
	}
	
}
