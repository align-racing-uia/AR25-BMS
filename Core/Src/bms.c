
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
void BroadcastBMSVoltages(BMS_HandleTypeDef *hbms);
void BroadcastBMSTemperatures(BMS_HandleTypeDef *hbms);

// Private variable defines
uint8_t voltage_cycle = 0; // This is used to cycle the voltage broadcast, so that it does not flood the bus
uint8_t temp_cycle = 0;    // This is used to cycle the temperature broadcast, so that it does not flood the bus

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
    hbms->SdcPin = hardware_config->SdcPin;             // Bind the SdcClosed pin
    hbms->TsRequested = false;                          // Initialize the TS requested flag to false

    HAL_GPIO_WritePin(hbms->FaultPin.Port, hbms->FaultPin.Pin, GPIO_PIN_SET);           // Set the fault pin high, to indicate no fault in the BMS
    HAL_GPIO_WritePin(hbms->PlusAIR.Port, hbms->PlusAIR.Pin, GPIO_PIN_RESET);           // Set the plus AIR pin low, to indicate no fault in the BMS
    HAL_GPIO_WritePin(hbms->MinusAIR.Port, hbms->MinusAIR.Pin, GPIO_PIN_RESET);         // Set the minus AIR pin low, to indicate no fault in the BMS
    HAL_GPIO_WritePin(hbms->PrechargeAIR.Port, hbms->PrechargeAIR.Pin, GPIO_PIN_RESET); // Set the precharge AIR pin low, to indicate no fault in the BMS

    hbms->CanTimestamp = HAL_GetTick();     // Initialize the CAN timestamp to the current time
    hbms->ChargerTimestamp = HAL_GetTick(); // Initialize the charger timestamp to the current time
    hbms->TempTimestamp = HAL_GetTick();    // Initialize the temperature timestamp to the current time
    hbms->VoltageTimestamp = HAL_GetTick(); // Initialize the voltage timestamp to the current time
    hbms->BroadcastTimestamp = HAL_GetTick(); // Initialize the broadcast timestamp to the current time
    hbms->ChargerPresent = false;           // Initialize the charger present flag to false

    hbms->BqConnected = false; // Initialize the BQ connected flag to false

    hbms->PackVoltage = &hbms->BQ->TotalVoltage; // Bind the pack voltage pointer
    hbms->SOC = &hbms->BatteryModel->EstimatedSOC; // Bind the SOC pointer
    hbms->DcLimit = 1200;
    hbms->CcLimit = 100;

    hbms->HighestCellTemperature = &hbms->BQ->HighestCellTemperature; // Bind the highest cell temperature pointer
    hbms->LowestCellTemperature = &hbms->BQ->LowestCellTemperature;   // Bind the lowest cell temperature pointer
    hbms->CellVoltages = hbms->BQ->CellVoltages;                      // Bind the cell voltages pointer
    hbms->CellTemperatures = hbms->BQ->CellTemperatures;              // Bind the cell temperatures pointer

    hbms->Initialized = true; // Clear the initialized flag
    hbms->BroadcastVoltages = false; // Clear the broadcast voltages flag
    hbms->BroadcastTemperatures = false; // Clear the broadcast temperatures flag
}

void BMS_Update(BMS_HandleTypeDef *hbms)
{

    // Things which are done before every cycle are done here
    CheckForFaults(hbms);
    ListenForCanMessages(hbms); // Listen for CAN messages

    hbms->SdcClosed = HAL_GPIO_ReadPin(hbms->SdcPin.Port, hbms->SdcPin.Pin) == GPIO_PIN_SET; // Read the SdcClosed pin to see if the SDC is closed

    if (hbms->BqConnected)
    {

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
    }

    uint16_t cycle_time = HAL_GetTick() - hbms->LastMeasurementTimestamp; // Calculate the cycle time
    hbms->LastMeasurementTimestamp = HAL_GetTick();                       // Update the last measurement timestamp
    // BatteryModel_Update(hbms->BatteryModel, hbms->BQ->CellVoltages, hbms->BQ->CellTemperatures, hbms->MeasuredCurrent, cycle_time);

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
            BQ_EnableCommTimeout(hbms->BQ);                     // Enable the BQ communication timeout
            BQ_StatusTypeDef status = BQ_EnableTsRef(hbms->BQ); // Enable the TS reference for the BQ
            if (status != BQ_STATUS_OK)
            {
                // If enabling the TS reference fails, set the state to fault
                SET_BIT(hbms->ActiveFaults, BMS_FAULT_BQ_NOT_CONNECTED); // Set the BQ TS reference error fault
                hbms->State = BMS_STATE_FAULT;
            }
            status = BQ_ConfigureMainADC(hbms->BQ); // Configure the main ADC for the BQ
            if (status != BQ_STATUS_OK)
            {
                // If enabling the TS reference fails, set the state to fault
                SET_BIT(hbms->ActiveFaults, BMS_FAULT_BQ_NOT_CONNECTED); // Set the BQ TS reference error fault
                hbms->State = BMS_STATE_FAULT;
            }
            status = BQ_ActivateMainADC(hbms->BQ); // Activate the main ADCs
            if (status != BQ_STATUS_OK)
            {
                // If enabling the TS reference fails, set the state to fault
                SET_BIT(hbms->ActiveFaults, BMS_FAULT_BQ_NOT_CONNECTED); // Set the BQ TS reference error fault
                hbms->State = BMS_STATE_FAULT;
            }

            hbms->State = BMS_STATE_IDLE; // Move to the idle state if the BQ is connected
        }
        else
        {
            // If the BQ is not connected, set the state to fault
            SET_BIT(hbms->ActiveFaults, BMS_FAULT_BQ_NOT_CONNECTED); // Set the BQ not connected fault
            hbms->State = BMS_STATE_FAULT;
        }
        break;
    }

    case BMS_STATE_IDLE: // Much of this functionality will be shared so we let it fall through

        if (hbms->TsRequested && hbms->SdcClosed)
        {
            hbms->State = BMS_STATE_TS_ACTIVE; // If the TS is requested (or we are connected to the charger) and the SDC is closed, we can activate the TS
        }

        if (hbms->ChargerPresent && hbms->SdcClosed)
        {
            hbms->State = BMS_STATE_CHARGING; // If the TS is requested (or we are connected to the charger) and the SDC is closed, we can activate the TS
        }

        break;
    // We let this fall through, as the charging state includes the TS active state
    case BMS_STATE_CHARGING:
        if (hbms->ChargerTimestamp + 1000 < HAL_GetTick())
        {
            // If the charger timestamp is older than 1 second, we consider the charger disconnected
            hbms->ChargerPresent = false; // Clear the charger present flag
        }
        // We let this fall through, as the charging state includes the TS active state

    case BMS_STATE_TS_ACTIVE:
        switch (hbms->TSState)
        {
        case TS_STATE_IDLE:
            // In the idle state, we can precharge the TS
            HAL_GPIO_WritePin(hbms->PrechargeAIR.Port, hbms->PrechargeAIR.Pin, GPIO_PIN_SET); // Set the precharge AIR pin high, to indicate precharging
            HAL_GPIO_WritePin(hbms->MinusAIR.Port, hbms->MinusAIR.Pin, GPIO_PIN_SET);         // Set the precharge AIR pin high, to indicate precharging
            hbms->TSState = TS_STATE_PRECHARGE;                                               // Move to the precharge state
            break;
        case TS_STATE_PRECHARGE:
            // In the precharge state, we can check if the precharge is complete
            if (hbms->ChargerPresent && hbms->SdcClosed)
            {
                // If the charger is present and the SDC is closed, we can activate the TS immediately
                HAL_GPIO_WritePin(hbms->PlusAIR.Port, hbms->PrechargeAIR.Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(hbms->MinusAIR.Port, hbms->MinusAIR.Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(hbms->PrechargeAIR.Port, hbms->PrechargeAIR.Pin, GPIO_PIN_RESET);
                hbms->TSState = TS_STATE_ACTIVE; // Move to the active state
            }

            if ((((float)hbms->InverterVoltage) >= (*hbms->PackVoltage) * 0.9f) && (*hbms->PackVoltage) > 300.0f)
            {
                // If the inverter voltage is above 90% of the pack voltage, we can activate the TS
                HAL_GPIO_WritePin(hbms->PlusAIR.Port, hbms->PrechargeAIR.Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(hbms->MinusAIR.Port, hbms->MinusAIR.Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(hbms->PrechargeAIR.Port, hbms->PrechargeAIR.Pin, GPIO_PIN_RESET);
                hbms->TSState = TS_STATE_ACTIVE; // Move to the active state
            }

            break;
        case TS_STATE_ACTIVE:
            if (!(hbms->TsRequested || hbms->ChargerPresent) || !hbms->SdcClosed || hbms->ActiveWarnings)
            {
                // If the TS is not requested or the SDC is not closed, we need to move to the idle state
                HAL_GPIO_WritePin(hbms->PlusAIR.Port, hbms->PlusAIR.Pin, GPIO_PIN_RESET);           // Set the plus AIR pin low, to indicate no TS active
                HAL_GPIO_WritePin(hbms->MinusAIR.Port, hbms->MinusAIR.Pin, GPIO_PIN_RESET);         // Set the minus AIR pin low, to indicate no TS active
                HAL_GPIO_WritePin(hbms->PrechargeAIR.Port, hbms->PrechargeAIR.Pin, GPIO_PIN_RESET); // Set the precharge AIR pin low, to indicate no TS active
                hbms->TsRequested = false;                                                          // Clear the TS requested flag
                hbms->TSState = TS_STATE_IDLE;                                                      // Move to the idle state
            }
            break;
        }

        break;

    case BMS_STATE_FAULT:

        // Send the BMS fault signal
        HAL_GPIO_WritePin(hbms->FaultPin.Port, hbms->FaultPin.Pin, GPIO_PIN_RESET); // Set the fault pin low, to indicate a fault in the BMS

        // Make properly sure that no relays are set (although the SDC should do the same)
        HAL_GPIO_WritePin(hbms->PlusAIR.Port, hbms->PlusAIR.Pin, GPIO_PIN_RESET);           // Set the plus AIR pin low, to indicate no TS active
        HAL_GPIO_WritePin(hbms->MinusAIR.Port, hbms->MinusAIR.Pin, GPIO_PIN_RESET);         // Set the minus AIR pin low, to indicate no TS active
        HAL_GPIO_WritePin(hbms->PrechargeAIR.Port, hbms->PrechargeAIR.Pin, GPIO_PIN_RESET); // Set the precharge AIR pin low, to indicate no TS active
        hbms->TSState = TS_STATE_IDLE;                                                      // Move to the idle state

        // Currently to reset the BMS, you need to power cycle it

        break;
    }

    // Things that need to happen regardless of the state
    if (hbms->BroadcastTimestamp + 100 <= HAL_GetTick())
    {
        // If the broadcast timestamp is older than 100ms, we need to broadcast the BMS state
        hbms->BroadcastTimestamp = HAL_GetTick(); // Update the broadcast timestamp

        BroadcastBMSState(hbms); // Broadcast the BMS state to the CAN bus

        if (hbms->BroadcastVoltages)
        {
            BroadcastBMSVoltages(hbms);
        }

        if (hbms->BroadcastTemperatures)
        {
            BroadcastBMSTemperatures(hbms);
        }
    }
}

// Function to monitor and update fault flags
// Certain ActiveFaults are set elsewere, such as BQ related faults
void CheckForFaults(BMS_HandleTypeDef *hbms)
{

    // All checks dependant on the BQ being connected
    if (hbms->BqConnected){
    // if (*hbms->HighestCellTemperature > 60.0f || ((*hbms->LowestCellTemperature < -20.0f) && (*hbms->LowestCellTemperature > -30.0f)))
    // {
    //     SET_BIT(hbms->ActiveFaults, BMS_FAULT_CRITICAL_TEMPERATURE); // Set the temperature fault
    // }

    // if (*hbms->LowestCellTemperature <= -30.0f)
    // {
    //     SET_BIT(hbms->ActiveFaults, BMS_FAULT_LOST_TEMPERATURE_SENSOR); // Set the temperature warning
    // }

    // if (*hbms->LowestCellVoltage < 2500.0f || *hbms->HighestCellVoltage > 4200.00f)
    // {
    //     // Currently no bit is set to indicate voltage faults, so we use the BMS_FAULT_BQ bit
    //     SET_BIT(hbms->ActiveFaults, BMS_FAULT_CRITICAL_VOLTAGE); // Set the voltage fault
    // }
    }

    // Set the relevant flags based on the faults
    if (hbms->ActiveFaults > 0)
    {
        // If there are any faults, set the state to fault
        hbms->State = BMS_STATE_FAULT;
    }
}

void CheckForWarnings(BMS_HandleTypeDef *hbms)
{
    // This function is currently not used, but can be used to check for warnings
    // and set the ActiveWarnings bitmask accordingly.
    // For now, we will just clear the ActiveWarnings bitmask.
    hbms->ActiveWarnings = 0; // Clear the active warnings

    if (hbms->CanTimestamp + 1000 < HAL_GetTick())
    {
        SET_BIT(hbms->ActiveWarnings, BMS_WARNING_CAN); // Set the CAN timeout fault
        // Charger cannot be present if the CAN is not working
        hbms->ChargerPresent = false; // Clear the charger present flag
    }
    // All checks dependant on the BQ being connected
    if (hbms->BqConnected)
    {
        if (*hbms->LowestCellVoltage <= 2800.0f)
        {
            SET_BIT(hbms->ActiveWarnings, BMS_WARNING_UNDERVOLTAGE); // Set the low cell voltage warning
        }

        if (*hbms->HighestCellTemperature > 59.0f)
        {
            SET_BIT(hbms->ActiveWarnings, BMS_WARNING_OVERTEMPERATURE); // Set the high temperature warning
        }
    }

    if (hbms->MeasuredCurrent > 120.0f)
    {
        SET_BIT(hbms->ActiveWarnings, BMS_WARNING_OVERCURRENT); // Set the high current warning
    }

    hbms->WarningPresent = hbms->ActiveWarnings > 0; // Set the warning present flag
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
        return false;
    }

    BQ_ClearComm(hbms->BQ);               // Clear the BQ communication buffer
    bq_status = BQ_AutoAddress(hbms->BQ); // Attempt to auto address the BQ
    if (bq_status != BQ_STATUS_OK)
    {
        // If the auto addressing fails, return false
        // This will set the state to fault in the main loop
        return false;
    }

    bq_status = BQ_ConfigureGPIO(hbms->BQ); // Configure the GPIOs of the BQ
    if (bq_status != BQ_STATUS_OK)
    {
        // If the GPIO configuration fails, return false
        // This will set the state to fault in the main loop
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
                if (node_id == 30)
                {
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

                // Dashboard Node ID
                if (node_id == 18)
                {
                    switch (packet_id)
                    {
                    case 0x1:
                        hbms->TsRequested = hbms->TsRequested || (((rx_data[0] & 0x02) > 0) && hbms->SdcClosed); // Set the TS requested flag based on the first byte of the received data
                        break;
                    }
                }

                if (node_id == hbms->Config.CanNodeID)
                {
                    switch (packet_id)
                    {
                    case 0x20:
                        hbms->BroadcastVoltages = (rx_data[0] & 0x01) > 0;     // Set the broadcast voltages flag based on the first byte of the received data
                        hbms->BroadcastTemperatures = (rx_data[0] & 0x02) > 0; // Set the broadcast temperatures flag based on the first byte of the received data
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
    data[1] = (uint8_t)hbms->DcLimit;      // Set the second byte to the BMS state
    data[2] = (uint8_t)hbms->CcLimit >> 8; // Set the third byte to the BMS state
    data[3] = (uint8_t)hbms->CcLimit;      // Set the fourth byte to the BMS state
    data[4] = (uint8_t)*hbms->HighestCellTemperature;
    data[5] = (uint8_t)*hbms->LowestCellTemperature;
    data[6] = (uint8_t)*hbms->SOC;

    Align_CAN_Send(hbms->FDCAN, Align_CombineCanId(0x1, hbms->Config.CanNodeID, hbms->Config.CanExtended), data, 7, hbms->Config.CanExtended); // Send the broadcast packet

    data[0] = (uint8_t)hbms->ActiveFaults;   // Set the first byte to the BMS state
    data[1] = (uint8_t)hbms->ActiveWarnings; // Set the second byte to the BMS state
    data[2] = (uint8_t)hbms->State;          // Set the third byte to the BMS state
    data[3] = (uint8_t)hbms->SdcClosed;      // Set the fourth byte to the BMS state

    Align_CAN_Send(hbms->FDCAN, Align_CombineCanId(0x2, hbms->Config.CanNodeID, hbms->Config.CanExtended), data, 4, hbms->Config.CanExtended); // Send the broadcast packet
}

void BroadcastBMSVoltages(BMS_HandleTypeDef *hbms)
{
    // This function transmits the cell voltages over the CAN network

    uint8_t data[8] = {0}; // Dummy data for the broadcast packet

    uint8_t total_cycles = hbms->Config.CellCount / 3 + (hbms->Config.CellCount % 3 > 0); // Calculate the total number of cycles needed to send all cell voltages
    data[0] = voltage_cycle;                                                              // Set the first byte to the current cycle index
    data[1] = total_cycles;                                                               // Set the second byte to the total number of cycles

    for (size_t i = 0; i < 3; i++)
    {
        if ((voltage_cycle * 3 + i) < hbms->Config.CellCount)
        {
            uint16_t low_res_voltage = (uint16_t)(hbms->BQ->CellVoltages[voltage_cycle * 3 + i]); // Convert the cell voltage to mV and store it in a low resolution format
            // If the current cycle index is within the range of cell voltages
            data[i * 2 + 2] = (uint8_t)(low_res_voltage >> 8); // Set the cell voltage in mV
            data[i * 2 + 3] = (uint8_t)(low_res_voltage);      // Set the cell voltage in mV
        }
        else
        {
            data[i * 2 + 2] = 0; // If the current cycle index is out of range, set the cell voltage to 0
            data[i * 2 + 3] = 0; // If the current cycle index is out of range, set the cell voltage to 0
        }
    }

    Align_CAN_Send(hbms->FDCAN, Align_CombineCanId(62, hbms->Config.CanNodeID, hbms->Config.CanExtended), data, 8, hbms->Config.CanExtended); // Send the broadcast packet
    voltage_cycle++;
    voltage_cycle = voltage_cycle % total_cycles; // Calculate the current cycle index
}

void BroadcastBMSTemperatures(BMS_HandleTypeDef *hbms)
{
    // This function transmits the cell temperatures over the CAN network

    uint8_t data[8] = {0}; // Dummy data for the broadcast packet

    // TODO: Toggle this based on if Multiplexing is enabled or not
    uint8_t total_cycles = (hbms->Config.NumOfSlaves * hbms->Config.TempsEach * (2)) / 3 + ((hbms->Config.NumOfSlaves * hbms->Config.TempsEach * (2)) % 3 > 0); // Calculate the total number of cycles needed to send all cell temperatures
    data[0] = temp_cycle;                                                                                                                                       // Set the first byte to the current cycle index
    data[1] = total_cycles;                                                                                                                                     // Set the second byte to the total number of cycles

    for (size_t i = 0; i < 3; i++)
    {
        if (temp_cycle * 3 + i < hbms->Config.CellCount)
        {
            uint16_t low_res_temperature = (uint16_t)(hbms->BQ->CellTemperatures[temp_cycle * 3 + i] * 10); // Convert the cell temperature to C and store it in a low resolution format
            // If the current cycle index is within the range of cell temperatures
            data[i * 2 + 2] = (uint8_t)(low_res_temperature >> 8); // Set the cell temperature in C
            data[i * 2 + 3] = (uint8_t)(low_res_temperature);      // Set the cell temperature in C
        }
        else
        {
            data[i * 2 + 2] = 0; // If the current cycle index is out of range, set the cell temperature to 0
            data[i * 2 + 3] = 0; // If the current cycle index is out of range, set the cell temperature to 0
        }
    }

    Align_CAN_Send(hbms->FDCAN, Align_CombineCanId(63, hbms->Config.CanNodeID, hbms->Config.CanExtended), data, 8, hbms->Config.CanExtended); // Send the broadcast packet
    temp_cycle++;
    temp_cycle = temp_cycle % total_cycles; // Calculate the current cycle index
}