#include "can.h"

void can_loopback_Init(void);                                         //初始化
void FloatToUint8(uint8_t *uintdata, float *data, uint16_t size);     // float转uint8_t
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanHandle); // CAN接收中断回调函数
