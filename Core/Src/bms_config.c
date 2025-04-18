#include "bms_config.h"
#include "w25q_mem.h"

BMS_Config_StatusTypeDef BMS_Config_WriteToFlash(BMS_Config_HandleTypeDef *bms_config){
    uint8_t* buffer = (uint8_t*)bms_config;
    uint16_t size = sizeof(BMS_Config_HandleTypeDef);
    size_t full_pages = size / 256;
    size_t remaining_bytes = size % 256;
    for (size_t i = 0; i < full_pages; i++)
    {
        if (W25Q_ProgramData(buffer + (i * 256), 256, 0, i) != W25Q_OK)
        {
            return BMS_CONFIG_ERROR;
        }
    }
    if(remaining_bytes > 0)
    {
        if (W25Q_ProgramData(buffer + (full_pages * 256), remaining_bytes, 0, full_pages) != W25Q_OK)
        {
            return BMS_CONFIG_ERROR;
        }
    }

    return BMS_Config_UpdateFromFlash(bms_config);
}

BMS_Config_StatusTypeDef BMS_Config_UpdateFromFlash(BMS_Config_HandleTypeDef *bms_config){
    uint8_t* buffer = (uint8_t*)bms_config;
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
    if(remaining_bytes > 0)
    {
        if (W25Q_ReadData(buffer + (full_pages * 256), remaining_bytes, 0, full_pages) != W25Q_OK)
        {
            return BMS_CONFIG_ERROR;
        }
    }

    if (strncmp(bms_config->MemoryCheck, "align", 5) == 0 || bms_config->ConfigVersion != BMS_CONFIG_VERSION)
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
BMS_Config_StatusTypeDef BMS_Config_HandleCanMessage(BMS_Config_HandleTypeDef *bms_config, uint16_t packet_id, uint8_t *data){

}