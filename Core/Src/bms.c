
#include "bms.h"
#include "stddef.h"

// Private Function defines
bool Initialize(BMS_HandleTypeDef *hbms);
bool Connect(BMS_HandleTypeDef *hbms);
bool Configure(BMS_HandleTypeDef *hbms);
void CheckForFaults(BMS_HandleTypeDef *hbms);

// Public Function implementations

void BMS_BindMemory(BMS_HandleTypeDef *hbms, BatteryModel_HandleTypeDef *battery_model, BQ_HandleTypeDef *bq)
{
    if (hbms == NULL || battery_model == NULL || bq == NULL)
    {
        // If this occurs, you have done something very wrong
        Error_Handler();
    }

    hbms->BatteryModel = battery_model;
    hbms->BQ = bq;
    hbms->State = BMS_STATE_BOOTING; // Initialize the state to booting
}

void BMS_Update(BMS_HandleTypeDef *hbms)
{

    CheckForFaults(hbms);

    switch (hbms->State)
    {
    case BMS_STATE_BOOTING:

        if (Initialize(hbms))
        {
            hbms->State = BMS_STATE_CONFIGURING; // Move to the next state
        }
        else
        {
            // If initialization fails, set the state to fault
            hbms->State = BMS_STATE_FAULT;
        }
        break;

    case BMS_STATE_CONFIGURING:
        if (Configure(hbms))
        {
            hbms->State = BMS_STATE_CONNECTING; // Move to the next state if configuration is successful
        }
        else
        {
            // If configuration fails, set the state to fault
            hbms->State = BMS_STATE_FAULT;
        }
        break;

    case BMS_STATE_CONNECTING:
        // Check if the BQ is connected
        if (Connect(hbms))
        {
            if(hbms->BQ->Connected)
            {
                // If the BQ is connected, move to the idle state
                hbms->State = BMS_STATE_IDLE; // Move to the idle state if the BQ is connected
            }else
            {
                // If the BQ is not connected, set the state to fault
                hbms->State = BMS_STATE_FAULT;
            }
        }
        else
        {
            // If the BQ is not connected, set the state to fault
            hbms->State = BMS_STATE_FAULT;
        }
        break;

    default:

        // Set the state to fault if it is not recognized
        // We need to know what state we are in at all times, for the system
        // to be deterministic
        hbms->State = BMS_STATE_FAULT;

        break;
    }
}

void CheckForFaults(BMS_HandleTypeDef *hbms)
{

    if (hbms == NULL)
    {
        // If this occurs, you have done something very wrong
        Error_Handler();
    }

    // Check the BQ for faults
    if (!hbms->BQ->Connected)
    {
        SET_BIT(hbms->ActiveFaults, BMS_FAULT_BQ); // Set the BQ fault flag if not connected
    }

    // Set the relevant flags based on the faults
    if (hbms->ActiveFaults & BMS_FAULT_MASK)
    {
        // If there are any faults, set the state to fault
        hbms->State = BMS_STATE_FAULT;
    }

    if (hbms->ActiveFaults & BMS_WARNING_MASK)
    {
        hbms->WarningPresent = true; // Set the warning present flag
    }
    else
    {
        hbms->WarningPresent = false; // Clear the warning present flag
    }
}

bool Connect(BMS_HandleTypeDef *hbms)
{
    // This function should implement the logic to connect to the BQ
    // For now, we will just return true to simulate a successful connection
    return true;
}

bool Configure(BMS_HandleTypeDef *hbms)
{
    // This function should implement the logic to configure the BQ
    // For now, we will just return true to simulate a successful configuration
    return true;
}

// Private Function implementations
bool Initialize(BMS_HandleTypeDef *hbms)
{

    return true;
}