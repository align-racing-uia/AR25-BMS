
#include "bms.h"
#include "stddef.h"
#include "w25q_mem.h"
#include "aligncan.h"
#include <stm32g4xx.h>

// #define BQ_DISABLE

// Private Function defines
bool Connect(BMS_HandleTypeDef *hbms);
bool LoadConfiguration(BMS_HandleTypeDef *hbms);
void UpdateFaultFlags(BMS_HandleTypeDef *hbms);
void ListenForCanMessages(BMS_HandleTypeDef *hbms);
void UpdateTSState(BMS_HandleTypeDef *hbms);

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
    hbms->TSState = TS_STATE_IDLE;       // Set the TS state to idle

    hbms->FDCAN = hardware_config->hfdcan;      // Bind the FDCAN handle from the hardware configuration
    hbms->FaultPin = hardware_config->FaultPin; // Bind the fault pin from the hardware configuration
    hbms->LowCurrentSensorPin = hardware_config->LowCurrentSensorPin; // Bind the low current sensor pin
    hbms->HighCurrentSensorPin = hardware_config->HighCurrentSensorPin; // Bind the high current sensor pin

    hbms->PlusAIR = hardware_config->PlusAIR; // Bind the plus AIR pin
    hbms->MinusAIR = hardware_config->MinusAIR; // Bind the minus AIR pin
    hbms->PrechargeAIR = hardware_config->PrechargeAIR; // Bind the precharge AIR pin

    HAL_GPIO_WritePin(hbms->FaultPin.Port, hbms->FaultPin.Pin, GPIO_PIN_SET); // Set the fault pin high, to indicate no fault in the BMS
    HAL_GPIO_WritePin(hbms->PlusAIR.Port, hbms->PlusAIR.Pin, GPIO_PIN_RESET); // Set the plus AIR pin low, to indicate no fault in the BMS
    HAL_GPIO_WritePin(hbms->MinusAIR.Port, hbms->MinusAIR.Pin, GPIO_PIN_RESET); // Set the minus AIR pin low, to indicate no fault in the BMS
    HAL_GPIO_WritePin(hbms->PrechargeAIR.Port, hbms->PrechargeAIR.Pin, GPIO_PIN_RESET); // Set the precharge AIR pin low, to indicate no fault in the BMS

    hbms->CanTimestamp = HAL_GetTick(); // Initialize the CAN timestamp to the current time
    hbms->ChargerTimestamp = HAL_GetTick(); // Initialize the charger timestamp to the current time
    hbms->ChargerPresent = false; // Initialize the charger present flag to false

    hbms->BqConnected = false; // Initialize the BQ connected flag to false

    hbms->PackCurrent = &hbms->MeasuredCurrent; // Bind the pack current pointer to the measured current, we do it like this to keep it consistent with the other pointers
    hbms->PackVoltage = &hbms->BatteryModel->PackVoltage; // Bind the pack voltage pointer
    hbms->AverageCellTemperature = &hbms->BatteryModel->AverageTemperature; // Bind the average cell temperature pointer
    hbms->AverageCellVoltage = &hbms->BatteryModel->AverageCellVoltage; // Bind the average cell voltage pointer

    hbms->HighestCellTemperature = &hbms->BQ->HighestCellTemperature; // Bind the highest cell temperature pointer
    hbms->LowestCellTemperature = &hbms->BQ->LowestCellTemperature; // Bind the lowest cell temperature pointer
    hbms->CellVoltages = hbms->BQ->CellVoltages; // Bind the cell voltages pointer
    hbms->CellTemperatures = hbms->BQ->CellTemperatures; // Bind the cell temperatures pointer

    hbms->Initialized = true; // Clear the initialized flag
}

void BMS_Update(BMS_HandleTypeDef *hbms)
{

    UpdateFaultFlags(hbms);
    ListenForCanMessages(hbms); // Listen for CAN messages

    switch (hbms->State)
    {

    case BMS_STATE_CONFIGURING:
    {
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
    }
    case BMS_STATE_CONNECTING:
    {    // Check if the BQ is connected
        if (Connect(hbms))
        {
            BQ_EnableCommTimeout(hbms->BQ); // Enable the BQ communication timeout
            hbms->State = BMS_STATE_IDLE; // Move to the idle state if the BQ is connected
        }
        else
        {
            // If the BQ is not connected, set the state to fault
            hbms->State = BMS_STATE_FAULT;
        }
        break;
    }
    case BMS_STATE_IDLE: // Much of this functionality will be shared
        BQ_GetCellVoltages(hbms->BQ); // Get the cell voltages from the BQ
        BQ_GetCellTemperatures(hbms->BQ, 1.0); // Get the cell temperatures from the BQ
    case BMS_STATE_DRIVING:
    case BMS_STATE_CHARGING:
        // In the charging state, we need to monitor the charger and the BQ
        if (hbms->ChargerPresent)
        {
            // If the charger is present, we can charge the battery
            hbms->State = BMS_STATE_CHARGING; // Stay in the charging state
        }
        else
        {
            // If the charger is not present, we need to move to the idle state
            hbms->State = BMS_STATE_IDLE;
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


void UpdateTSState(BMS_HandleTypeDef *hbms)
{
    // Unsure if this is the right way to go
    // TODO: See if the separate TS state machine is needed
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



    if (hbms->CanTimestamp + 1000 < HAL_GetTick())
    {
        SET_BIT(hbms->ActiveFaults, BMS_WARNING_CAN); // Set the CAN timeout fault
    }
    else
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

#if defined(BQ_DISABLE)
    return true; // If BQ is disabled, return true, so other parts of the code can still run    
#endif
    
    // This function should implement the logic to connect to the BQ
    // For now, we will just return true to simulate a successful connection

    BQ_StatusTypeDef bq_status; // Attempt to connect to the BQ
    BQ_WakePing(hbms->BQ);      // Wake up the BQ
    BQ_WakePing(hbms->BQ);      // Wake up the BQ

    bq_status = BQ_WakeMsg(hbms->BQ); // Send the wake message to the BQ subdevices

    if (bq_status != BQ_STATUS_OK)
    {
        // If the BQ is not connected, return false
        // This will set the state to fault in the main loop
        SET_BIT(hbms->ActiveFaults, BMS_FAULT_BQ); // Set the BQ fault flag
        return false;
    }

    BQ_ClearComm(hbms->BQ);               // Clear the BQ communication buffer
    bq_status = BQ_AutoAddress(hbms->BQ); // Attempt to auto address the BQ
    if (bq_status != BQ_STATUS_OK)
    {
        // If the auto addressing fails, return false
        // This will set the state to fault in the main loop
        SET_BIT(hbms->ActiveFaults, BMS_FAULT_BQ); // Set the BQ fault flag
        return false;
    }

    bq_status = BQ_ConfigureGPIO(hbms->BQ); // Configure the GPIOs of the BQ
    if (bq_status != BQ_STATUS_OK)
    {
        // If the GPIO configuration fails, return false
        // This will set the state to fault in the main loop
        SET_BIT(hbms->ActiveFaults, BMS_FAULT_BQ); // Set the BQ fault flag
        return false;
    }

    bq_status = BQ_ActivateSlaveADC(hbms->BQ); // Activate the slave ADCs
    if (bq_status != BQ_STATUS_OK)
    {
        // If the slave ADC activation fails, return false
        // This will set the state to fault in the main loop
        SET_BIT(hbms->ActiveFaults, BMS_FAULT_BQ); // Set the BQ fault flag
        return false;
    }
    // If all the above steps are successful, we can consider the BQ connected
    hbms->BqConnected = true; // Set the BQ connected flag to true
    return true;
}

bool LoadConfiguration(BMS_HandleTypeDef *hbms)
{

    // Initialize the EEPROM memory
    W25Q_STATE w25q_state = W25Q_Init();
    hbms->EepromPresent = w25q_state == W25Q_OK;

    if (BMS_Config_UpdateFromFlash(&hbms->Config) != BMS_CONFIG_OK) // Update the configuration from flash memory
    {
        // If this fails, it can still indicate that the EEPROM is present
        // But the configuration is invalid or corrupted
        BMS_Config_Init(&hbms->Config); // Reinitialize the configuration to default values
        if (BMS_Config_WriteToFlash(&hbms->Config) != BMS_CONFIG_OK)
        {
            // If the configuration write to flash fails, we set the EepromPresent flag to false
            // Note: The written config is also re-read from flash
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

    if (!hbms->EepromPresent)
    {
        SET_BIT(hbms->ActiveFaults, BMS_NOTE_EEPROM); // Set the EEPROM fault flag
    }

    return true;
}

void ListenForCanMessages(BMS_HandleTypeDef *hbms)
{

    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8]; // Buffer for the received CAN data

    uint8_t max_cycles = 5; // Maximum number of CAN messages to process in one main loop iteration

    while (max_cycles > 0 && Align_CAN_Receive(hbms->FDCAN, &rx_header, rx_data))
    {
        uint32_t can_id = rx_header.Identifier & 0x7FF; // Extract the CAN ID from the header
        uint16_t packet_id;
        uint16_t node_id;
        Align_SplitCanId(can_id, &packet_id, &node_id, rx_header.IdType == FDCAN_EXTENDED_ID); // Split the CAN ID into the packet ID

        // First check for exact matches of the CAN ID
        switch (can_id)
        {
        case 0x18FF50E5:
            hbms->ChargerPresent = true;            // Charger is present
            hbms->ChargerTimestamp = HAL_GetTick(); // Update the charger timestamp
            break;                                  // This is the ID from the charger
        default:
            // If no exact match is found, we can check for the node ID
            {
                if (node_id == hbms->Config.CanNodeID)
                {
                    // This is the BMS node ID, and its not sent by the BMS itself
                    // It is related to the BMS configuration
                    // We can process the packet based on the packet ID
                    switch (packet_id)
                    {
                    case 0x63:
                    {                                                                  // This is used to set configuration parameters directly
                        uint8_t config_param_id1 = rx_data[0];                         // The first byte is the configuration parameter ID
                        uint16_t config_param_value1 = (rx_data[1] << 8) | rx_data[2]; // The next two bytes are the configuration parameter value

                        BMS_Config_SetParameter(&hbms->Config, config_param_id1, config_param_value1); // Set the configuration parameter in the BMS configuration

                        Align_CAN_Send(hbms->FDCAN, Align_CombineCanId(0x62, hbms->Config.CanNodeID, rx_header.IdType == FDCAN_EXTENDED_ID), rx_data, 6, rx_header.IdType == FDCAN_EXTENDED_ID); // Send the message back to the CAN bus to acknowledge the change

                        break;
                    }
                    case 0x01:
                        // This tells the BMS to write the configuration to flash memory, and reset
                        if (BMS_Config_WriteToFlash(&hbms->Config) == BMS_CONFIG_OK)
                        {
                            // If the configuration write is successful, reset the BMS
                            NVIC_SystemReset(); // Reset the system
                        }
                        else
                        {
                            // If the configuration write fails, set the EEPROM fault flag
                            SET_BIT(hbms->ActiveFaults, BMS_NOTE_EEPROM); // Set the EEPROM fault flag
                        }
                    default:
                        // Unknown packet ID, ignore it
                        break;
                    }
                }
            }
            break;
        }
    }
}