#include "ts_statemachine.h"
#include "stddef.h"
#include "stdbool.h"
#include "faults.h"

void TS_Init(TS_HandleTypeDef *hts){
    if (hts == NULL)
    {
        // If this occurs, you have done something very wrong
        Error_Handler();
    }
    
    hts->State = TS_STATE_IDLE; // Initialize the state to idle
}

void TS_UpdateState(TS_HandleTypeDef *hts, float soc, bool sdc, bool ts_enable, bool charger_connected, BMS_FaultFlags fault_flags){
    if (hts == NULL)
    {
        // If this occurs, you have done something very wrong
        Error_Handler();
    }

    if(fault_flags & BMS_FAULT_MASK & BMS_WARNING_MASK)
    {
        hts->State = TS_STATE_FAULT; // Set the state to fault if there are any faults
    }

    switch(hts->State)
    {
        case TS_STATE_IDLE:
            break;
        case TS_STATE_PRECHARGE:
            // Do nothing
            break;
        case TS_STATE_ACTIVE:
            // Do nothing
            break;
        case TS_STATE_FAULT:
            // Do nothing
            break;
        default:
            // Invalid state, do nothing
            break;
    }
}