#include "can.h"

void can_loopback_Init(void);                                         //��ʼ��
void FloatToUint8(uint8_t *uintdata, float *data, uint16_t size);     // floatתuint8_t
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanHandle); // CAN�����жϻص�����
