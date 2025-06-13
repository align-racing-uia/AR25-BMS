
#include "bms.h"
#include "stddef.h"
#include "w25q_mem.h"
#include "aligncan.h"
#include "faults.h"
#include <stm32g4xx.h>

// #define BQ_DISABLE

// Private Function defines
bool Connect(BMS_HandleTypeDef *hbms);
bool LoadConfiguration(BMS_HandleTypeDef *hbms);
void CheckForFaults(BMS_HandleTypeDef *hbms);
void ListenForCanMessages(BMS_HandleTypeDef *hbms);
void BroadcastBMSState(BMS_HandleTypeDef *hbms);

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

    hbms->FDCAN = hardware_config->hfdcan;                              // Bind the FDCAN handle from the hardware configuration
    hbms->FaultPin = hardware_config->FaultPin;                         // Bind the fault pin from the hardware configuration
    hbms->LowCurrentSensorPin = hardware_config->LowCurrentSensorPin;   // Bind the low current sensor pin
    hbms->HighCurrentSensorPin = hardware_config->HighCurrentSensorPin; // Bind the high current sensor pin

    hbms->PlusAIR = hardware_config->PlusAIR;           // Bind the plus AIR pin
    hbms->MinusAIR = hardware_config->MinusAIR;         // Bind the minus AIR pin
    hbms->PrechargeAIR = hardware_config->PrechargeAIR; // Bind the precharge AIR pin
    hbms->SdcPin = hardware_config->SdcPin; // Bind the SdcClosed pin
    hbms->TsRequested = false; // Initialize the TS requested flag to false

    HAL_GPIO_WritePin(hbms->FaultPin.Port, hbms->FaultPin.Pin, GPIO_PIN_SET);           // Set the fault pin high, to indicate no fault in the BMS
    HAL_GPIO_WritePin(hbms->PlusAIR.Port, hbms->PlusAIR.Pin, GPIO_PIN_RESET);           // Set the plus AIR pin low, to indicate no fault in the BMS
    HAL_GPIO_WritePin(hbms->MinusAIR.Port, hbms->MinusAIR.Pin, GPIO_PIN_RESET);         // Set the minus AIR pin low, to indicate no fault in the BMS
    HAL_GPIO_WritePin(hbms->PrechargeAIR.Port, hbms->PrechargeAIR.Pin, GPIO_PIN_RESET); // Set the precharge AIR pin low, to indicate no fault in the BMS

    hbms->CanTimestamp = HAL_GetTick();     // Initialize the CAN timestamp to the current time
    hbms->ChargerTimestamp = HAL_GetTick(); // Initialize the charger timestamp to the current time
    hbms->TempTimestamp = HAL_GetTick();    // Initialize the temperature timestamp to the current time
    hbms->VoltageTimestamp = HAL_GetTick(); // Initialize the voltage timestamp to the current time
    hbms->ChargerPresent = false;           // Initialize the charger present flag to false

    hbms->BqConnected = false; // Initialize the BQ connected flag to false

    hbms->PackVoltage = &hbms->BQ->TotalVoltage;                   // Bind the pack voltage pointer
    
    hbms->HighestCellTemperature = &hbms->BQ->HighestCellTemperature; // Bind the highest cell temperature pointer
    hbms->LowestCellTemperature = &hbms->BQ->LowestCellTemperature;   // Bind the lowest cell temperature pointer
    hbms->CellVoltages = hbms->BQ->CellVoltages;                      // Bind the cell voltages pointer
    hbms->CellTemperatures = hbms->BQ->CellTemperatures;              // Bind the cell temperatures pointer

    hbms->Initialized = true; // Clear the initialized flag
}

void BMS_Update(BMS_HandleTypeDef *hbms)
{

    // Things which are done before every cycle are done here
    CheckForFaults(hbms);
    ListenForCanMessages(hbms); // Listen for CAN messages

    hbms->SdcClosed = HAL_GPIO_ReadPin(hbms->SdcPin.Port, hbms->SdcPin.Pin) == GPIO_PIN_SET; // Read the SdcClosed pin to see if the SDC is closed

    if (hbms->VoltageTimestamp + 5 < HAL_GetTick())
    {
        // If the voltage timestamp is older than 5ms, we need to update the cell voltages
        BQ_GetCellVoltages(hbms->BQ);           // Get the cell voltages from the BQ
        hbms->VoltageTimestamp = HAL_GetTick(); // Update the voltage timestamp
    }

    if (hbms->TempTimestamp + 100 < HAL_GetTick())
    {
        BQ_GetCellTemperatures(hbms->BQ, 4300.0); // Get the cell temperatures from the BQ
        hbms->TempTimestamp = HAL_GetTick();      // Update the temperature timestamp
    }

    uint16_t cycle_time = HAL_GetTick() - hbms->LastMeasurementTimestamp; // Calculate the cycle time
    hbms->LastMeasurementTimestamp = HAL_GetTick(); // Update the last measurement timestamp
    BatteryModel_Update(hbms->BatteryModel, hbms->BQ->CellVoltages, hbms->BQ->CellTemperatures, hbms->MeasuredCurrent, cycle_time);

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
    { // Check if the BQ is connected
        if (Connect(hbms))
        {
            BQ_EnableCommTimeout(hbms->BQ); // Enable the BQ communication timeout
            BQ_EnableTsRef(hbms->BQ);       // Enable the TS reference for the BQ
            BQ_ConfigureMainADC(hbms->BQ); // Configure the main ADC for the BQ
            BQ_ActivateMainADC(hbms->BQ);  // Activate the main ADCs

            hbms->State = BMS_STATE_IDLE; // Move to the idle state if the BQ is connected
        }
        else
        {
            // If the BQ is not connected, set the state to fault
            hbms->State = BMS_STATE_FAULT;
        }
        break;
    }

    case BMS_STATE_IDLE: // Much of this functionality will be shared so we let it fall through

    if (hbms->TsRequested && hbms->SdcClosed){
        hbms->State = BMS_STATE_TS_ACTIVE; // If the TS is requested (or we are connected to the charger) and the SDC is closed, we can activate the TS
    }

    if (hbms->ChargerPresent && hbms->SdcClosed){
        hbms->State = BMS_STATE_CHARGING; // If the TS is requested (or we are connected to the charger) and the SDC is closed, we can activate the TS
    }

    break;
    // We let this fall through, as the charging state includes the TS active state
    case BMS_STATE_CHARGING:
        if(hbms->ChargerTimestamp + 1000 < HAL_GetTick())
        {
            // If the charger timestamp is older than 1 second, we consider the charger disconnected
            hbms->ChargerPresent = false; // Clear the charger present flag
        }
    // We let this fall through, as the charging state includes the TS active state

    case BMS_STATE_TS_ACTIVE:
        switch(hbms->TSState){
            case TS_STATE_IDLE:
                // In the idle state, we can precharge the TS
                HAL_GPIO_WritePin(hbms->PrechargeAIR.Port, hbms->PrechargeAIR.Pin, GPIO_PIN_SET); // Set the precharge AIR pin high, to indicate precharging
                HAL_GPIO_WritePin(hbms->MinusAIR.Port, hbms->MinusAIR.Pin, GPIO_PIN_SET); // Set the precharge AIR pin high, to indicate precharging
                hbms->TSState = TS_STATE_PRECHARGE; // Move to the precharge state
                break;
            case TS_STATE_PRECHARGE:
                // In the precharge state, we can check if the precharge is complete
                if(hbms->ChargerPresent && hbms->SdcClosed)
                {
                    // If the charger is present and the SDC is closed, we can activate the TS immediately
                    HAL_GPIO_WritePin(hbms->PlusAIR.Port, hbms->PrechargeAIR.Pin, GPIO_PIN_SET); 
                    HAL_GPIO_WritePin(hbms->MinusAIR.Port, hbms->MinusAIR.Pin, GPIO_PIN_SET); 
                    HAL_GPIO_WritePin(hbms->PrechargeAIR.Port, hbms->PrechargeAIR.Pin, GPIO_PIN_RESET);
                    hbms->TSState = TS_STATE_ACTIVE; // Move to the active state
                }
                
                if((((float) hbms->InverterVoltage) >= (*hbms->PackVoltage) * 0.9f) && (*hbms->PackVoltage) > 300.0f)
                {
                    // If the inverter voltage is above 90% of the pack voltage, we can activate the TS
                    HAL_GPIO_WritePin(hbms->PlusAIR.Port, hbms->PrechargeAIR.Pin, GPIO_PIN_SET); 
                    HAL_GPIO_WritePin(hbms->MinusAIR.Port, hbms->MinusAIR.Pin, GPIO_PIN_SET); 
                    HAL_GPIO_WritePin(hbms->PrechargeAIR.Port, hbms->PrechargeAIR.Pin, GPIO_PIN_RESET);
                    hbms->TSState = TS_STATE_ACTIVE; // Move to the active state
                }
                
                break;
            case TS_STATE_ACTIVE:
                if(!(hbms->TsRequested || hbms->ChargerPresent) || !hbms->SdcClosed)
                {
                    // If the TS is not requested or the SDC is not closed, we need to move to the idle state
                    HAL_GPIO_WritePin(hbms->PlusAIR.Port, hbms->PlusAIR.Pin, GPIO_PIN_RESET); // Set the plus AIR pin low, to indicate no TS active
                    HAL_GPIO_WritePin(hbms->MinusAIR.Port, hbms->MinusAIR.Pin, GPIO_PIN_RESET); // Set the minus AIR pin low, to indicate no TS active
                    HAL_GPIO_WritePin(hbms->PrechargeAIR.Port, hbms->PrechargeAIR.Pin, GPIO_PIN_RESET); // Set the precharge AIR pin low, to indicate no TS active
                    hbms->TsRequested = false; // Clear the TS requested flag
                    hbms->TSState = TS_STATE_IDLE; // Move to the idle state
                }
                break;

        }

    break;

    case BMS_STATE_FAULT:

        // Send the BMS fault signal
        HAL_GPIO_WritePin(hbms->FaultPin.Port, hbms->FaultPin.Pin, GPIO_PIN_RESET); // Set the fault pin low, to indicate a fault in the BMS
        HAL_GPIO_WritePin(hbms->PlusAIR.Port, hbms->PlusAIR.Pin, GPIO_PIN_RESET); // Set the plus AIR pin low, to indicate no TS active
        HAL_GPIO_WritePin(hbms->MinusAIR.Port, hbms->MinusAIR.Pin, GPIO_PIN_RESET); // Set the minus AIR pin low, to indicate no TS active
        HAL_GPIO_WritePin(hbms->PrechargeAIR.Port, hbms->PrechargeAIR.Pin, GPIO_PIN_RESET); // Set the precharge AIR pin low, to indicate no TS active
        hbms->TSState = TS_STATE_IDLE; // Move to the idle state

        // Currently to reset the BMS, you need to power cycle it

        break;
    }

    // Things that need to happen regardless of the state
    BroadcastBMSState(hbms); // Broadcast the BMS state to the CAN bus
}


// Function to monitor and update fault flags
// Certain ActiveFaults are set elsewere, such as BQ related faults
void CheckForFaults(BMS_HandleTypeDef *hbms)
{

    if (hbms == NULL)
    {
        // If this occurs, you have done something very wrong
        Error_Handler();
    }

    if (hbms->CanTimestamp + 1000 < HAL_GetTick())
    {
        SET_BIT(hbms->ActiveFaults, BMS_WARNING_CAN); // Set the CAN timeout fault
        // Charger cannot be present if the CAN is not working
        hbms->ChargerPresent = false; // Clear the charger present flag
    }
    else
    {
        CLEAR_BIT(hbms->ActiveFaults, BMS_WARNING_CAN); // Clear the CAN timeout fault
    }


    if(*hbms->HighestCellTemperature > 59.0f || *hbms->LowestCellTemperature < -20.0f)
    {
        SET_BIT(hbms->ActiveFaults, BMS_FAULT_TEMP); // Set the temperature fault
        hbms->State = BMS_STATE_FAULT; // If the highest cell temperature is above 59C or the lowest cell temperature is below -20C, set the state to fault
    }

    if(*hbms->LowestCellVoltage < 2800.0f || *hbms->HighestCellVoltage > 4150.00f)
    {
        // Currently no bit is set to indicate voltage faults, so we use the BMS_FAULT_BQ bit
        SET_BIT(hbms->ActiveFaults, BMS_FAULT_BQ); // Set the voltage fault
        hbms->State = BMS_STATE_FAULT; // If the lowest cell voltage is below 2.5V or the highest cell voltage is above 4.5V, set the state to fault
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
        // TODO: Add the GpioAuxADCMap and FirstTempGPIO to the BMS configuration, as well as a multiplexing toggle
        .TempMultiplexEnabled = true,
        .TempMultiplexPinIndex = 7, // Pin 8 (zero indexed) is used to multiplex the temperature sensors, which is GPIO7 in the BQ
        .GpioAuxADCMap = 0x7F,
        .FirstTempGPIO = 1, // Pin 1 is used to detect PCB temperature, and the rest are used for the temperature sensors
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
                // Inverter Node ID
                // TODO: Make this a configurable field...
                if(node_id == 30){
                    // This is the Inverter Node ID, and its not sent by the BMS itself
                    // It is related to the Inverter configuration
                    // We can process the packet based on the packet ID
                    switch (packet_id)
                    {
                    case 0x20:
                        hbms->InverterVoltage = (rx_data[6] << 8) | rx_data[7]; // Set the inverter voltage from the received data
                        break;
                    }
                }
                if (node_id == hbms->Config.CanConfigNodeID)
                {
                    // This is the BMS Config node ID, and its not sent by the BMS itself
                    // It is related to the BMS configuration
                    // We can process the packet based on the packet ID
                    BMS_Config_HandleCanMessage(&hbms->Config, packet_id, rx_data); // Handle the BMS configuration CAN message
                }
            }
            break;
        }
    }
}

void BroadcastBMSState(BMS_HandleTypeDef *hbms)
{
    // This function transmits the state of the BMS over the CAN network, as well as some limits


    uint8_t data[8] = {0}; // Dummy data for the broadcast packet

    data[0] = (uint8_t)hbms->DcLimit >> 8; // Set the first byte to the BMS state
    data[1] = (uint8_t)hbms->DcLimit; // Set the second byte to the BMS state
    data[2] = (uint8_t)hbms->CcLimit >> 8; // Set the third byte to the BMS state
    data[3] = (uint8_t)hbms->CcLimit; // Set the fourth byte to the BMS state
    data[4] = (uint8_t) *hbms->HighestCellTemperature;
    data[5] = (uint8_t) *hbms->LowestCellTemperature;
    data[6] = (uint8_t) *hbms->SOC;

    Align_CAN_Send(hbms->FDCAN, Align_CombineCanId(0x1, hbms->Config.CanNodeID, hbms->Config.CanExtended), data, 7, hbms->Config.CanExtended); // Send the broadcast packet

    data[0] = (uint8_t)hbms->ActiveFaults; // Set the first byte to the BMS state
    data[1] = (uint8_t)hbms->SdcClosed; // Set the second byte to the BMS state

    Align_CAN_Send(hbms->FDCAN, Align_CombineCanId(0x2, hbms->Config.CanNodeID, hbms->Config.CanExtended), data, 2, hbms->Config.CanExtended); // Send the broadcast packet


}