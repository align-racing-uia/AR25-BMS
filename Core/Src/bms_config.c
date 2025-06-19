#include "bms_config.h"
#include "w25q_mem.h"

// ====== BMS CONFIG DEFAULTS ======
#define BMS_CONFIG_VERSION 0
#define BMS_CONFIG_DEFAULT_NUM_OF_SLAVES 10
#define BMS_CONFIG_DEFAULT_CELLS_EACH 14
#define BMS_CONFIG_DEFAULT_TEMPS_EACH 6
#define BMS_CONFIG_DEFAULT_FIRST_TEMP_PIN_INDEX 1
#define BMS_CONFIG_DEFAULT_MULTIPLEX_PIN_INDEX 7
#define BMS_CONFIG_DEFAULT_MULTIPLEX_ENABLED true

#define BMS_CONFIG_DEFAULT_CELLS_IN_PARALLEL 5
#define BMS_CONFIG_DEFAULT_CELL_VOLTAGE_LIMIT_LOW 2700
#define BMS_CONFIG_DEFAULT_CELL_VOLTAGE_LIMIT_HIGH 4200
#define BMS_CONFIG_DEFAULT_CELL_VOLTAGE_DERATE_LIMIT_LOW 3000
#define BMS_CONFIG_DEFAULT_CELL_VOLTAGE_DERATE_LIMIT_HIGH 4100
#define BMS_CONFIG_DEFAULT_CELL_TEMPERATURE_LIMIT_LOW -20.0f
#define BMS_CONFIG_DEFAULT_CELL_TEMPERATURE_LIMIT_HIGH 60.0f
#define BMS_CONFIG_DEFAULT_CELL_TEMPERATURE_DERATE_LIMIT_LOW 0.0f
#define BMS_CONFIG_DEFAULT_CELL_TEMPERATURE_DERATE_LIMIT_HIGH 50.0f
#define BMS_CONFIG_DEFAULT_CELL_DISCHARGE_CURRENT_LIMIT 40000 // 4000 mA = 4 A
#define BMS_CONFIG_DEFAULT_CELL_CHARGE_CURRENT_LIMIT 2800 // 2800 mA = 2.8 A
#define BMS_CONFIG_DEFAULT_SINGLE_CELL_CAPACITY 0

#define BMS_CONFIG_DEFAULT_FUSE_CURRENT_LIMIT 200000 // 200000 mA = 200 A, this is the fuse current limit for the BMS, this is the maximum current that can flow through the fuse continuously

#define BMS_CONFIG_DEFAULT_CAN_NODE_ID 6
#define BMS_CONFIG_DEFAULT_CAN_CONFIG_NODE_ID 7
#define BMS_CONFIG_DEFAULT_CAN_SPEED BMS_Config_CAN_SPEED_500K // Default CAN speed is 500k, this is the baudrate of the CAN bus
#define BMS_CONFIG_DEFAULT_BROADCAST_PACKET_ID 1
#define BMS_CONFIG_DEFAULT_CAN_BROADCAST_INTERVAL 10
#define BMS_CONFIG_DEFAULT_CAN_TEMP_BROADCAST_INTERVAL 10
#define BMS_CONFIG_DEFAULT_CAN_VOLTAGE_BROADCAST_INTERVAL 10
#define BMS_CONFIG_DEFAULT_CAN_CHARGER_BROADCAST_INTERVAL 100
#define BMS_CONFIG_DEFAULT_CAN_CHARGER_BROADCAST_TIMEOUT 2000
#define BMS_CONFIG_DEFAULT_CAN_EXTENDED false

void BMS_Config_Init(BMS_Config_HandleTypeDef *bms_config)
{
    if (bms_config == NULL)
    {
        return;
    }

    bms_config->ConfigVersion = BMS_CONFIG_VERSION;
    bms_config->NumOfSlaves = BMS_CONFIG_DEFAULT_NUM_OF_SLAVES;
    bms_config->CellsEach = BMS_CONFIG_DEFAULT_CELLS_EACH;
    bms_config->TempsEach = BMS_CONFIG_DEFAULT_TEMPS_EACH;
    bms_config->CellCount = bms_config->NumOfSlaves * bms_config->CellsEach;
    bms_config->FirstTempPinIndex = BMS_CONFIG_DEFAULT_FIRST_TEMP_PIN_INDEX;
    bms_config->MultiplexPinIndex = BMS_CONFIG_DEFAULT_MULTIPLEX_PIN_INDEX;
    bms_config->MultiplexEnabled = BMS_CONFIG_DEFAULT_MULTIPLEX_ENABLED;

    bms_config->CellsInParallel = BMS_CONFIG_DEFAULT_CELLS_IN_PARALLEL;
    bms_config->CellVoltageLimitLow = BMS_CONFIG_DEFAULT_CELL_VOLTAGE_LIMIT_LOW;
    bms_config->CellVoltageLimitHigh = BMS_CONFIG_DEFAULT_CELL_VOLTAGE_LIMIT_HIGH;
    bms_config->CellVoltageDerateLimitLow = BMS_CONFIG_DEFAULT_CELL_VOLTAGE_DERATE_LIMIT_LOW;
    bms_config->CellVoltageDerateLimitHigh = BMS_CONFIG_DEFAULT_CELL_VOLTAGE_DERATE_LIMIT_HIGH;
    bms_config->CellTemperatureLimitLow = BMS_CONFIG_DEFAULT_CELL_TEMPERATURE_LIMIT_LOW;
    bms_config->CellTemperatureLimitHigh = BMS_CONFIG_DEFAULT_CELL_TEMPERATURE_LIMIT_HIGH;
    bms_config->CellTemperatureDerateLimitLow = BMS_CONFIG_DEFAULT_CELL_TEMPERATURE_DERATE_LIMIT_LOW;
    bms_config->CellTemperatureDerateLimitHigh = BMS_CONFIG_DEFAULT_CELL_TEMPERATURE_DERATE_LIMIT_HIGH;
    bms_config->CellDischargeCurrentLimit = BMS_CONFIG_DEFAULT_CELL_DISCHARGE_CURRENT_LIMIT;
    bms_config->CellChargeCurrentLimit = BMS_CONFIG_DEFAULT_CELL_CHARGE_CURRENT_LIMIT;
    bms_config->SingleCellCapacity = BMS_CONFIG_DEFAULT_SINGLE_CELL_CAPACITY;

    bms_config->FuseCurrentLimit = BMS_CONFIG_DEFAULT_FUSE_CURRENT_LIMIT;

    bms_config->CanNodeID = BMS_CONFIG_DEFAULT_CAN_NODE_ID;
    bms_config->CanConfigNodeID = BMS_CONFIG_DEFAULT_CAN_CONFIG_NODE_ID;
    bms_config->CanSpeed = BMS_CONFIG_DEFAULT_CAN_SPEED;
    bms_config->BroadcastPacketID = BMS_CONFIG_DEFAULT_BROADCAST_PACKET_ID;
    bms_config->CanBroadcastInterval = BMS_CONFIG_DEFAULT_CAN_BROADCAST_INTERVAL;
    bms_config->CanTempBroadcastInterval = BMS_CONFIG_DEFAULT_CAN_TEMP_BROADCAST_INTERVAL;
    bms_config->CanVoltageBroadcastInterval = BMS_CONFIG_DEFAULT_CAN_VOLTAGE_BROADCAST_INTERVAL;
    bms_config->CanChargerBroadcastInterval = BMS_CONFIG_DEFAULT_CAN_CHARGER_BROADCAST_INTERVAL;
    bms_config->CanChargerBroadcastTimeout = BMS_CONFIG_DEFAULT_CAN_CHARGER_BROADCAST_TIMEOUT;
    bms_config->CanExtended = BMS_CONFIG_DEFAULT_CAN_EXTENDED;

    bms_config->MemoryCheck[0] = 'a';
    bms_config->MemoryCheck[1] = 'l';
    bms_config->MemoryCheck[2] = 'i';
    bms_config->MemoryCheck[3] = 'g';
    bms_config->MemoryCheck[4] = 'n';
    bms_config->Checksum = 0;
}

// Set a parameter in the configuration, index is the parameter index, value is the value to set
// TODO: Implement more parameters
void BMS_Config_SetParameter(BMS_Config_HandleTypeDef *bms_config, BMS_Config_ParameterIndexTypeDef index, uint16_t value)
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
    if (bms_config->NumOfSlaves == 0 || bms_config->CellsEach == 0 || bms_config->TempsEach == 0 || ((uint16_t)bms_config->CellsEach * (uint16_t)bms_config->NumOfSlaves) > 255)
    {
        return BMS_CONFIG_INVALID_VALUE;
    }

    if (bms_config->CellVoltageLimitLow > bms_config->CellVoltageLimitHigh)
    {
        return BMS_CONFIG_INVALID_VALUE;
    }

    return BMS_CONFIG_OK;
}

BMS_Config_StatusTypeDef BMS_Config_HandleCanMessage(BMS_Config_HandleTypeDef *bms_config, uint16_t packet_id, uint8_t *can_data)
{
}