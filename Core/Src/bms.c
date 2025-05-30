
#include "bms.h"
#include "stddef.h"
#include "w25q_mem.h"
#include "aligncan.h"

// Private Function defines
bool Connect(BMS_HandleTypeDef *hbms);
bool LoadConfiguration(BMS_HandleTypeDef *hbms);
void UpdateFaultFlags(BMS_HandleTypeDef *hbms);
void ListenForCanMessages(BMS_HandleTypeDef *hbms);

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
    hbms->State = BMS_STATE_CONFIGURING; // Initialize the state to booting
}

void BMS_Init(BMS_HandleTypeDef *hbms, BMS_HardwareConfigTypeDef *hardware_config)
{
    if (hbms == NULL)
    {
        // If this occurs, you have done something very wrong
        Error_Handler();
    }

    hbms->State = BMS_STATE_CONFIGURING; // Set the initial state to configuring
    hbms->ActiveFaults = 0;              // Clear the active faults
    hbms->WarningPresent = false;        // Clear the warning present flag
    hbms->SdcClosed = false;             // Clear the SdcClosed connected flag
    hbms->EepromPresent = false;         // Clear the EEPROM present flag
    TS_Init(hbms->TS);                   // Initialize the TS state machine

    hbms->FaultPin = hardware_config->FaultPin;          // Bind the fault pin from the hardware configuration
    hbms->FDCAN = hardware_config->hfdcan;              // Bind the FDCAN handle from the hardware configuration

    hbms->Initialized = true; // Clear the initialized flag
}

void BMS_Update(BMS_HandleTypeDef *hbms)
{

    UpdateFaultFlags(hbms);
    ListenForCanMessages(hbms); // Listen for CAN messages

    switch (hbms->State)
    {

    case BMS_STATE_CONFIGURING:
        if (LoadConfiguration(hbms))
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
            if (hbms->BQ->Connected)
            {
                // If the BQ is connected, move to the idle state
                hbms->State = BMS_STATE_IDLE; // Move to the idle state if the BQ is connected
            }
            else
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
    
    case BMS_STATE_FAULT:
        
        // Send the BMS fault signal
        HAL_GPIO_WritePin(hbms->FaultPin.Port, hbms->FaultPin.Pin, GPIO_PIN_RESET); // Set the fault pin low, to indicate a fault in the BMS
        break;
    default:

        // Set the state to fault if it is not recognized
        // We need to know what state we are in at all times, for the system
        // to be deterministic
        hbms->State = BMS_STATE_FAULT;

        break;
    }
}

// Function to monitor and update fault flags
// Certain ActiveFaults are set elsewere, such as BQ related faults
void UpdateFaultFlags(BMS_HandleTypeDef *hbms)
{

    if (hbms == NULL)
    {
        // If this occurs, you have done something very wrong
        Error_Handler();
    }

    // Check if charger is still connected, timeout after 1 second, i dont believe this needs to be configurable
    if(hbms->ChargerPresent && ((hbms->ChargerTimestamp + 1000) < HAL_GetTick()))
    {
        hbms->ChargerPresent = false; // Clear the charger present flag if the timeout has passed
    }

    if(hbms->CanTimestamp + 1000 < HAL_GetTick())
    {
        SET_BIT(hbms->ActiveFaults, BMS_WARNING_CAN); // Set the CAN timeout fault
    }else
    {
        CLEAR_BIT(hbms->ActiveFaults, BMS_WARNING_CAN); // Clear the CAN timeout fault
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

// Private Function implementations

bool Connect(BMS_HandleTypeDef *hbms)
{
    // This function should implement the logic to connect to the BQ
    // For now, we will just return true to simulate a successful connection
    return true;
}

bool LoadConfiguration(BMS_HandleTypeDef *hbms)
{

    // Initialize the EEPROM memory
    W25Q_STATE w25q_state = W25Q_Init();
    hbms->EepromPresent = w25q_state == W25Q_OK;

    BMS_Config_Init(&hbms->Config); // Initialize the BMS configuration

    if (!hbms->EepromPresent)
    {
        // If the EEPROM is not present, we cannot update the configuration from flash
        // But we use default values for the configuration
        // This is indicated by the EepromPresent flag
        return true;
    }

    if (BMS_Config_UpdateFromFlash(&hbms->Config) == BMS_CONFIG_OK) // Update the configuration from flash memory
    {
        // If the configuration update is successful, we can proceed
        // This will also set the ConfigVersion to the correct value
        return true;
    }
    else
    {
        // If this fails, it can still indicate that the EEPROM is present
        // But the configuration is invalid or corrupted
        BMS_Config_Init(&hbms->Config); // Reinitialize the configuration to default values
        if (BMS_Config_WriteToFlash(&hbms->Config) != BMS_CONFIG_OK)
        {
            // If the configuration write to flash fails, we set the EepromPresent flag to false
            // Note: The written config is also re-read from flash
            SET_BIT(hbms->ActiveFaults, BMS_NOTE_EEPROM); // Set the EEPROM fault flag
            hbms->EepromPresent = false; // Set the EEPROM present flag to false
        }
    }

    // Configure the BQ with the BMS configuration
    BQ_ConfigTypeDef bq_config = {
        .NumOfSlaves = hbms->Config.NumOfSlaves,
        .NumOfCellsEach = hbms->Config.CellsEach,
        .NumOfTempsEach = hbms->Config.TempsEach,
        // TODO: Add the GpioAuxADCMap and CellTempPinMap to the BMS configuration, as well as a multiplexing toggle
        .TempMultiplexEnabled = true,
        .TempMultiplexPinIndex = 7, // Pin 8 (zero indexed) is used to multiplex the temperature sensors, which is GPIO7 in the BQ
        .GpioAuxADCMap = 0x7E,
        .CellTempPinMap = 0x7F, // Pin 1 is used to detect PCB temperature, and the rest are used for the temperature sensors
    };

    BQ_Configure(hbms->BQ, &bq_config); // Configure the BQ with the BMS configuration

    // Configure the battery model with the BMS configuration
    // TODO: Make capacity a bms configuration parameter
    BatteryModel_Configure(hbms->BatteryModel, hbms->Config.CellCount, hbms->Config.CellsEach * hbms->Config.NumOfSlaves, hbms->Config.CellCountInParallel, 2650); // Initialize the battery model with the configuration

    // Load the OCV maps from flash memory and set them in the battery model
    // TODO: Load the OCV maps

    // Apply values to the BQ
    BQ_Init(hbms->BQ); // Initialize the BQ

    return true;
}

void ListenForCanMessages(BMS_HandleTypeDef *hbms)
{

    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8]; // Buffer for the received CAN data

    uint8_t max_cycles = 5; // Maximum number of CAN messages to process in one main loop iteration

    while (max_cycles > 0 && Align_CAN_Receive(hbms->FDCAN, &rx_header, rx_data) == HAL_OK)
    {
        uint32_t can_id = rx_header.Identifier & 0x7FF; // Extract the CAN ID from the header
        uint16_t packet_id;
        uint16_t node_id;
        Align_SplitCanId(can_id, &packet_id, &node_id, rx_header.IdType == FDCAN_EXTENDED_ID); // Split the CAN ID into the packet ID

        switch (node_id)
        {


            default:
                // If the node ID is not recognized, check if it is a regular ID we know
                switch(can_id){
                    case 0x18FF50E5:
                        hbms->ChargerPresent = true; // Charger is present
                        hbms->ChargerTimestamp = HAL_GetTick(); // Update the charger timestamp
                        break; // This is the ID from the charger
                }
                break;
        }
    }
}