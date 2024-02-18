#include "bsp_can.h"

/**
 * @brief       CANͨ�ſ�������
 * @param       hcan��CANͨ�ž��
 * @retval      void
 * @note        �����ȳ�ʼ��CANͨ���˲��������ٿ���CANͨ��
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
 * @brief       CANͨ���˲�����
 * @param       hcan��CANͨ�ž��
 * @retval      void
 * @note        ����CANͨ�Ž���ʱ�������ʼ���ú��������齫�ú�����CANͨ�ſ���ǰ�ͱ���ʼ��
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
	  if(HAL_CAN_ConfigFilter(hcan, &CAN_Filter_st)!=HAL_OK)       //�˲�����ʼ��
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
	  if(HAL_CAN_ConfigFilter(hcan, &CAN_Filter_st)!=HAL_OK)        //�˲�����ʼ��
		{
			Error_Handler();
		}
	}
	
}
