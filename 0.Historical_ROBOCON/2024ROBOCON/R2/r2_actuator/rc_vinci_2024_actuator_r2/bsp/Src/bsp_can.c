#include "bsp_can.h"

extern CAN_HandleTypeDef hcan1;
// extern CAN_HandleTypeDef hcan2;

void CAN_Start(CAN_HandleTypeDef *hcan)
{
  HAL_CAN_Start(hcan);

  if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
  }
}

void CAN1_Filter_Init(void)
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
  HAL_CAN_ConfigFilter(&hcan1, &CAN_Filter_st); // ÂË²¨Æ÷³õÊ¼»¯
}

// void CAN2_Filter_Init(void)
//{
//	  CAN_FilterTypeDef CAN_Filter_st;
//     CAN_Filter_st.FilterActivation = ENABLE;
//     CAN_Filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
//     CAN_Filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
//     CAN_Filter_st.FilterIdHigh = 0x0000;
//     CAN_Filter_st.FilterIdLow = 0x0000;
//     CAN_Filter_st.FilterMaskIdHigh = 0x0000;
//     CAN_Filter_st.FilterMaskIdLow = 0x0000;
//     CAN_Filter_st.FilterBank = 14;
//		CAN_Filter_st.SlaveStartFilterBank = 14;
//     CAN_Filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
//	  HAL_CAN_ConfigFilter(&hcan2, &CAN_Filter_st);        //ÂË²¨Æ÷³õÊ¼»¯
// }
