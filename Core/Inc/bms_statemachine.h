#ifndef BMS_STATEMACHINE_H
#define BMS_STATEMACHINE_H

#include "faults.h"
#include "stdbool.h"

typedef enum
{
    BMS_STATE_BOOTING,
    BMS_STATE_IDLE,
    BMS_STATE_ACTVATING_TS,
    BMS_STATE_CHARGING,
    BMS_STATE_DRIVING,
    BMS_STATE_FAULT
} BMS_StateTypeDef;

typedef struct
{
    BMS_StateTypeDef State;      // The state of the BMS
    BMS_FaultFlags ActiveFaults; // Active faults bitmask
    bool WarningPresent;         // Warning present flag
    bool FaultPresent;           // Fault present flag
    bool SDC;                    // SDC connected flag
} BMS_HandleTypeDef;

void BMS_Init(BMS_HandleTypeDef *hbms);
void BMS_UpdateState(BMS_HandleTypeDef *hbms);

#endif // BMS_STATEMACHINE_H