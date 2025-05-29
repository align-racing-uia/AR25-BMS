#ifndef TS_STATEMACHINE_H
#define TS_STATEMACHINE_H

#include "stdbool.h"
#include "faults.h"

typedef enum
{
  TS_STATE_IDLE = 0,
  TS_STATE_PRECHARGE = 1,
  TS_STATE_ACTIVE = 2,
  TS_STATE_FAULT = 3,
} TS_StateTypeDef;

typedef struct 
{
    TS_StateTypeDef State; // The state of the TS
} TS_HandleTypeDef;

void TS_Init(TS_HandleTypeDef *hts);
void TS_UpdateState(TS_HandleTypeDef *hts, float soc, bool sdc, bool ts_enable, bool charger_connected, BMS_FaultFlags fault_flags);


#endif // TS_STATEMACHINE_H