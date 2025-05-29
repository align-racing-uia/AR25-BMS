
#include "bms.h"
#include "stddef.h"

// Private Function defines
bool Initialize(BMS_HandleTypeDef *hbms);

void BMS_Init(BMS_HandleTypeDef *hbms, BatteryModel_HandleTypeDef *battery_model, TS_HandleTypeDef *ts, BQ_HandleTypeDef *bq, BMS_Config_HandleTypeDef *config)
{
    if (hbms == NULL || battery_model == NULL || ts == NULL || bq == NULL || config == NULL)
    {
        // If this occurs, you have done something very wrong
        Error_Handler();
    }

    hbms->BatteryModel = battery_model;
    hbms->TS = ts;
    hbms->BQ = bq;
    hbms->Config = config;
    hbms->State = BMS_STATE_INITIALIZING; // Initialize the state to booting
}
void BMS_Update(BMS_HandleTypeDef *hbms)
{
    if (hbms->ActiveFaults & BMS_FAULT_MASK)
    {
        // If there are any faults, set the state to fault
        hbms->State = BMS_STATE_FAULT;
    }

    if(hbms->ActiveFaults & BMS_WARNING_MASK)
    {
        hbms->WarningPresent = true; // Set the warning present flag
    }else{
        hbms->WarningPresent = false; // Clear the warning present flag
    }

    switch (hbms->State)
    {
    case BMS_STATE_INITIALIZING:
        
        if(Initialize(hbms))
        {
            hbms->State = BMS_STATE_CONNECTING; // Move to the next state
        }
        else
        {
            // If initialization fails, stay in the same state
            hbms->State = BMS_STATE_FAULT;
        }
        break;
    
    case BMS_STATE_CONFIGURING:
        break;

    default:
        break;
    }
}

// Private Function implementations
bool Initialize(BMS_HandleTypeDef *hbms)
{
    // Perform all the initialization steps here
    // This is a placeholder function, you can add your own initialization code here
    // For now, we will just return true to indicate that the initialization was successful
    return true;
}