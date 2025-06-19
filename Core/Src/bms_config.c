#include "bms_config.h"
#include "w25q_mem.h"


void BMS_Config_Init(BMS_Config_HandleTypeDef *bms_config)
{
    // Setting the default values of the config
    bms_config->ConfigVersion = BMS_CONFIG_VERSION;
    bms_config->MemoryCheck[0] = 'a';
    bms_config->MemoryCheck[1] = 'l';
    bms_config->MemoryCheck[2] = 'i';
    bms_config->MemoryCheck[3] = 'g';
    bms_config->MemoryCheck[4] = 'n';

    bms_config->NumOfSlaves = DEFAULT_TOTAL_CHIPS - 1; // The master is not counted as a slave
    bms_config->CellsEach = DEFAULT_CELLS_EACH;
    bms_config->TempsEach = DEFAULT_TEMPS_EACH;
    bms_config->CellVoltageLimitLow = DEFAULT_CELLVOLTAGE_LIMIT_LOW;
    bms_config->CellVoltageLimitHigh = DEFAULT_CELLVOLTAGE_LIMIT_HIGH;
    bms_config->CellTemperatureLimitLow = DEFAULT_CELLTEMPERATURE_LIMIT_LOW;   // -40C
    bms_config->CellTemperatureLimitHigh = DEFAULT_CELLTEMPERATURE_LIMIT_HIGH; // 85C
    bms_config->CanNodeID = DEFAULT_CAN_NODE_ID;
    bms_config->CanExtended = DEFAULT_CAN_EXTENDED; // Should the CAN ID be extended or not
    bms_config->CanBroadcastInterval = DEFAULT_CAN_BROADCAST_INTERVAL;                // 100ms
    bms_config->CanTempBroadcastInterval = DEFAULT_CAN_TEMP_BROADCAST_INTERVAL;       // 1s
    bms_config->CanVoltageBroadcastInterval = DEFAULT_CAN_TEMP_BROADCAST_INTERVAL;    // 1s
    bms_config->CanChargerBroadcastInterval = DEFAULT_CAN_CHARGER_BROADCAST_INTERVAL; // 1s
    bms_config->CanChargerBroadcastTimeout = DEFAULT_CAN_CHARGER_BROADCAST_TIMEOUT;   // 5s
    bms_config->Checksum = 0x00;                                                      // TODO: Implement CRC checksum
}


// Set a parameter in the configuration, index is the parameter index, value is the value to set
// TODO: Implement more parameters
void BMS_Config_SetParameter(BMS_Config_HandleTypeDef *bms_config, uint8_t index, uint16_t value)
{

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
    if (bms_config->NumOfSlaves == 0 || bms_config->CellsEach == 0 || bms_config->TempsEach == 0)
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
BMS_Config_StatusTypeDef BMS_Config_HandleCanMessage(BMS_Config_HandleTypeDef *bms_config, uint16_t packet_id, uint8_t *can_data)
{
    
}