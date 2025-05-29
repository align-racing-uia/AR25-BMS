
#include "bms_statemachine.h"
#include "stddef.h"

void BMS_Init(BMS_HandleTypeDef *hbms){
    if (hbms == NULL)
    {
        // If this occurs, you have done something very wrong
        Error_Handler();
    }
    
    hbms->State = BMS_STATE_BOOTING; // Initialize the state to booting
}
void BMS_UpdateState(BMS_HandleTypeDef *hbms){
    
}