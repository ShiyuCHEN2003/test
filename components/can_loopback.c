#include "can_loopback.h"
#include "math.h"

void Configure_Filter(void);

void can_loopback_Init(void)
{

	//	Configure_Filter();
	//	HAL_CAN_Start(&hcan);
	//	HAL_CAN_ActivateNotification(&hcan,CAN_IT_RX_FIFO0_MSG_PENDING);
	Configure_Filter(); //����������  STM32CubeMX�Զ����ɵĴ�����û�У���Ҫ�Լ�����
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); //���������ж�
}

//�˲���
void Configure_Filter(void)
{
	CAN_FilterTypeDef Filter;
	Filter.FilterIdHigh = 0X0000;
	Filter.FilterIdLow = 0X0000;
	Filter.FilterMaskIdHigh = 0X0000;
	Filter.FilterMaskIdLow = 0X0000;
	Filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	Filter.FilterBank = 1;
	Filter.FilterMode = CAN_FILTERMODE_IDMASK;
	Filter.FilterScale = CAN_FILTERSCALE_32BIT;
	Filter.FilterActivation = ENABLE;
	Filter.SlaveStartFilterBank = 14;
	if (HAL_CAN_ConfigFilter(&hcan1, &Filter) != HAL_OK)
		Error_Handler();
}

// floatתuint8����
void FloatToUint8(uint8_t *uintdata, float *data, uint16_t size)
{
	uint16_t i;
	for (i = 0; i < size; i++)
	{
		uintdata[i] = round(data[i]);
	}
}
