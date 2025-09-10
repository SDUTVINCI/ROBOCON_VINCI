#include "bsp_fdcan.h"


/**
 * @brief       FDCAN通信开启函数
 * @param       hcan：FDCAN通信句柄
 * @retval      void
 * @note        建议先初始化FDCAN通信滤波函数，再开启CAN通信
 */
void CAN_BUS::BSP::FDCAN_Start(FDCAN_HandleTypeDef *hfdcan)
{
	if(HAL_FDCAN_Start(hfdcan) != HAL_OK)
	{
		Error_Handler();
	}

	if(HAL_FDCAN_ActivateNotification(hfdcan,FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief       FDCAN通信滤波函数
 * @param       hcan：FDCAN通信句柄
 * @retval      void
 * @note        开启FDCAN通信接收时，必须初始化该函数，建议将该函数在FDCAN通信开启前就被初始化
 */
void CAN_BUS::BSP::Filter_Init(FDCAN_HandleTypeDef *hfdcan)
{
	if(hfdcan->Instance == FDCAN1)	
	{
		FDCAN_FilterTypeDef  FDCAN_Filter_st;
		FDCAN_Filter_st.IdType = FDCAN_STANDARD_ID;//标准ID 
		FDCAN_Filter_st.FilterIndex = 0;			//滤波器索引
		FDCAN_Filter_st.FilterType = FDCAN_FILTER_MASK; // 使用掩码过滤
		FDCAN_Filter_st.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;//过滤器0关联到FIFO0  
		FDCAN_Filter_st.FilterID1 = 0x200; // 起始ID，位于 0x200
		FDCAN_Filter_st.FilterID2 = 0x7F8; // 掩码忽略低3位（2^3=8

	    if(HAL_FDCAN_ConfigFilter(hfdcan, &FDCAN_Filter_st)!=HAL_OK)       //滤波器初始化
		{
				Error_Handler();
		}
		//
		if (HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_ACCEPT_IN_RX_FIFO0,// 非匹配标准帧允许进 FIFO0
			FDCAN_REJECT,										 // 非匹配扩展帧bu允许进 FIFO0
			FDCAN_FILTER_REMOTE, 
			FDCAN_FILTER_REMOTE) != HAL_OK)
		{
		   	Error_Handler();
		}
		//开启 FIFO0 水位线中断通知,水位线阈值（0~63），表示当 FIFO 中的消息数达到这个值时触发中断
		if (HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_RX_FIFO0, 1) != HAL_OK) 
		{
			Error_Handler();
		}
	}

//	if(hfdcan->Instance == FDCAN2)	
//	{
//		FDCAN_FilterTypeDef  FDCAN_Filter_st;
//		FDCAN_Filter_st.IdType = FDCAN_STANDARD_ID;//标准ID 
//		FDCAN_Filter_st.FilterIndex = 0;			//滤波器索引
//		FDCAN_Filter_st.FilterType = FDCAN_FILTER_MASK; // 使用掩码过滤
//		FDCAN_Filter_st.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;//过滤器0关联到FIFO0  
//		FDCAN_Filter_st.FilterID1 = 0x200; // 起始ID，位于 0x200
//		FDCAN_Filter_st.FilterID2 = 0x7F8; // 掩码忽略低3位（2^3=8

//	    if(HAL_FDCAN_ConfigFilter(hfdcan, &FDCAN_Filter_st)!=HAL_OK)       //滤波器初始化
//		{
//				Error_Handler();
//		}
//		//
//		if (HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_ACCEPT_IN_RX_FIFO0,// 非匹配标准帧允许进 FIFO0
//			FDCAN_REJECT,										 // 非匹配扩展帧bu允许进 FIFO0
//			FDCAN_FILTER_REMOTE, 
//			FDCAN_FILTER_REMOTE) != HAL_OK)
//		{
//		   	Error_Handler();
//		}
//		//开启 FIFO0 水位线中断通知,水位线阈值（0~63），表示当 FIFO 中的消息数达到这个值时触发中断
//		if (HAL_FDCAN_ConfigFifoWatermark(hfdcan, FDCAN_RX_FIFO0, 1) != HAL_OK) 
//		{
//			Error_Handler();
//		}
//	}
	
}
