#ifndef ALIGN_CAN_H
#define ALIGN_CAN_H
#include "fdcan.h"
#include "stdint.h"
#include "align.h"
#include "stdbool.h"

typedef enum {
    ALIGN_CAN_SPEED_500KBPS,
    ALIGN_CAN_SPEED_1MBPS,
} Align_CAN_Speed_Typedef;

Align_StatusTypedef Align_CAN_Init(FDCAN_HandleTypeDef *hfdcan, Align_CAN_Speed_Typedef speed,  FDCAN_GlobalTypeDef* fdcan_instance);

Align_StatusTypedef Align_CAN_Send(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *data, uint8_t len);

bool Align_CAN_Receive(FDCAN_HandleTypeDef *hfdcan, FDCAN_RxHeaderTypeDef *rxHeader, uint8_t *rxData);


#endif // ALIGN_CAN_H