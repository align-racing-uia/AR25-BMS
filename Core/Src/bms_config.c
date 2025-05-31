#include "bms_config.h"
#include "w25q_mem.h"

typedef enum
{
    BMS_CONFIG_PARAM_VERSION = 0, // Config version, this should increment on large config changes
    BMS_CONFIG_PARAM_CELL_COUNT = 1, // Total number of cells
    BMS_CONFIG_PARAM_NUM_OF_SLAVES = 2, // The number of slaves in the system
} BMS_ConfigParameters;

void BMS_Config_Init(BMS_Config_HandleTypeDef *bms_config)
{
    // Setting the default values of the config
    bms_config->ConfigVersion = BMS_CONFIG_VERSION;
    bms_config->MemoryCheck[0] = 'a';
    bms_config->MemoryCheck[1] = 'l';
    bms_config->MemoryCheck[2] = 'i';
    bms_config->MemoryCheck[3] = 'g';
    bms_config->MemoryCheck[4] = 'n';
    bms_config->BroadcastPacket = DEFAULT_CAN_BROADCAST_PACKET;
    bms_config->CellCount = DEFAULT_TOTAL_CELLS;
    bms_config->NumOfChips = DEFAULT_TOTAL_CHIPS;
    bms_config->NumOfSlaves = DEFAULT_TOTAL_CHIPS - 1; // The master is not counted as a slave
    bms_config->CellsEach = DEFAULT_CELLS_EACH;
    bms_config->TempsEach = DEFAULT_TEMPS_EACH;
    bms_config->TempMapVoltagePoints = DEFAULT_TEMP_MAP_VOLTAGE_POINTS;
    bms_config->TempMapAmount = DEFAULT_TEMP_MAP_AMOUNT;
    bms_config->TotalCellCountInSeries = DEFAULT_TOTAL_CELLS_IN_SERIES;
    bms_config->CellCountInParallel = DEFAULT_CELLS_IN_PARALLEL;
    bms_config->CellVoltageLimitLow = DEFAULT_CELLVOLTAGE_LIMIT_LOW;
    bms_config->CellVoltageLimitHigh = DEFAULT_CELLVOLTAGE_LIMIT_HIGH;
    bms_config->CellTemperatureLimitLow = DEFAULT_CELLTEMPERATURE_LIMIT_LOW;   // -40C
    bms_config->CellTemperatureLimitHigh = DEFAULT_CELLTEMPERATURE_LIMIT_HIGH; // 85C
    bms_config->CanNodeID = DEFAULT_CAN_NODE_ID;
    bms_config->CanVoltageNodeID = DEFAULT_CAN_CELLVOLTAGE_NODE_ID;
    bms_config->CanTemperatureNodeID = DEFAULT_CAN_CELLTEMPERATURE_NODE_ID;
    bms_config->CanBaudrate = DEFAULT_CAN_BAUDRATE;
    bms_config->CanExtended = DEFAULT_CAN_EXTENDED; // Should the CAN ID be extended or not
    bms_config->UsbLoggingEnabled = DEFAULT_USB_LOGGING_ENABLED;
    bms_config->CanBroadcastInterval = DEFAULT_CAN_BROADCAST_INTERVAL;                // 100ms
    bms_config->CanTempBroadcastInterval = DEFAULT_CAN_TEMP_BROADCAST_INTERVAL;       // 1s
    bms_config->CanVoltageBroadcastInterval = DEFAULT_CAN_TEMP_BROADCAST_INTERVAL;    // 1s
    bms_config->CanTempBroadcastEnabled = true;                                       // Should the BMS broadcast the temperature or not
    bms_config->CanVoltageBroadcastEnabled = true;                                    // Should the BMS broadcast the temperature or not
    bms_config->UsbLoggingInterval = DEFAULT_USB_LOGGING_INTERVAL;                    // 1s
    bms_config->CanChargerBroadcastInterval = DEFAULT_CAN_CHARGER_BROADCAST_INTERVAL; // 1s
    bms_config->CanChargerBroadcastTimeout = DEFAULT_CAN_CHARGER_BROADCAST_TIMEOUT;   // 5s
    bms_config->BalanceWhileCharging = DEFAULT_BALANCE_WHILE_CHARGING;                // Should the BMS balance while charging or not
    bms_config->Checksum = 0x00;                                                      // TODO: Implement CRC checksum
}


// Set a parameter in the configuration, index is the parameter index, value is the value to set
// TODO: Implement more parameters
void BMS_Config_SetParameter(BMS_Config_HandleTypeDef *bms_config, uint8_t index, uint16_t value)
{
    switch(index){
        case BMS_CONFIG_PARAM_VERSION:
            bms_config->ConfigVersion = value;
            break;
        case BMS_CONFIG_PARAM_CELL_COUNT:
            bms_config->CellCount = value;
            bms_config->TotalCellCountInSeries = value / bms_config->CellsEach;
            bms_config->CellCountInParallel = 1; // TODO: Implement parallel cells
            break;
        case BMS_CONFIG_PARAM_NUM_OF_SLAVES:
            bms_config->NumOfSlaves = value;
            break;
        default:
            // Invalid parameter index, do nothing
            break;
    }
}

BMS_Config_StatusTypeDef BMS_Config_WriteToFlash(BMS_Config_HandleTypeDef *bms_config)
{
    uint8_t *buffer = (uint8_t *)bms_config;
    uint16_t size = sizeof(BMS_Config_HandleTypeDef);
    size_t full_pages = size / 256;
    size_t remaining_bytes = size % 256;
    if (W25Q_EraseSector(0) != W25Q_OK)
    {
        return BMS_CONFIG_ERROR;
    }
    for (size_t i = 0; i < full_pages; i++)
    {
        if (W25Q_ProgramData(buffer + (i * 256), 256, 0, i) != W25Q_OK)
        {
            return BMS_CONFIG_ERROR;
        }
    }
    if (remaining_bytes > 0)
    {
        if (W25Q_ProgramData(buffer + (full_pages * 256), remaining_bytes, 0, full_pages) != W25Q_OK)
        {
            return BMS_CONFIG_ERROR;
        }
    }

    return BMS_Config_UpdateFromFlash(bms_config);
}

BMS_Config_StatusTypeDef BMS_Config_UpdateFromFlash(BMS_Config_HandleTypeDef *bms_config)
{
    uint8_t *buffer = (uint8_t *)bms_config;
    uint16_t size = sizeof(BMS_Config_HandleTypeDef);
    size_t full_pages = size / 256;
    size_t remaining_bytes = size % 256;
    for (size_t i = 0; i < full_pages; i++)
    {
        if (W25Q_ReadData(buffer + (i * 256), 256, 0, i) != W25Q_OK)
        {
            return BMS_CONFIG_ERROR;
        }
    }
    if (remaining_bytes > 0)
    {
        if (W25Q_ReadData(buffer + (full_pages * 256), remaining_bytes, 0, full_pages) != W25Q_OK)
        {
            return BMS_CONFIG_ERROR;
        }
    }
    uint8_t string_check = strncmp(bms_config->MemoryCheck, "align", 5);
    if (string_check != 0 || bms_config->ConfigVersion != BMS_CONFIG_VERSION)
    {
        return BMS_CONFIG_INVALID_CONFIG;
    }
    if (bms_config->NumOfSlaves == 0 || bms_config->CellCount == 0 || bms_config->CellsEach == 0 || bms_config->TempsEach == 0)
    {
        return BMS_CONFIG_INVALID_VALUE;
    }
    if (bms_config->CellCount > CELL_MEMORY_POOL_SIZE)
    {
        return BMS_CONFIG_INVALID_VALUE;
    }
    if (bms_config->TempMapAmount > TEMP_MAP_POOL_AMOUNT)
    {
        return BMS_CONFIG_INVALID_VALUE;
    }
    if (bms_config->TempMapVoltagePoints > TEMP_MAP_POOL_MAX_POINTS)
    {
        return BMS_CONFIG_INVALID_VALUE;
    }
    if (bms_config->CellVoltageLimitLow > bms_config->CellVoltageLimitHigh)
    {
        return BMS_CONFIG_INVALID_VALUE;
    }

    // TODO: Implement CRC checksum

    return BMS_CONFIG_OK;
}
BMS_Config_StatusTypeDef BMS_Config_HandleCanMessage(BMS_Config_HandleTypeDef *bms_config, uint16_t packet_id, uint8_t *data)
{
}