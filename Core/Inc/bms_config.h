#ifndef __BMS_CONFIG_H
#define __BMS_CONFIG_H

#include "stdint.h"
#include "stddef.h"
#include "stdbool.h"

typedef enum
{
    BMS_Config_CAN_SPEED_100K = 0,  // 100 kbit/s
    BMS_Config_CAN_SPEED_125K = 1,  // 125 kbit/s
    BMS_Config_CAN_SPEED_250K = 2,  // 250 kbit/s
    BMS_Config_CAN_SPEED_500K = 3,  // 500 kbit/s
    BMS_Config_CAN_SPEED_1000K = 4, // 1000 kbit/s
} BMS_Config_CanSpeedTypeDef;

// All of these parameters can be set through serial, and will be stored on the flash
typedef struct
{
    uint16_t ConfigVersion;    // This number will automatically increment when the config changes
    uint8_t NumOfSlaves;       // The number of slaves in the system
    uint8_t CellsEach;         // The number of cells in series measured on each slave chip, this is used to calculate the total number of cells in series
    uint8_t TempsEach;         // The number of temperature sensors on each slave chip (before multiplexing), this is used to calculate the total number of temperature sensors in the system
    uint8_t CellCount;         // The total number of cells in series, this is calculated as NumOfSlaves * CellsEach
    uint8_t FirstTempPinIndex; // The first GPIO pin on the BQ79616 that is used for temperature sensors
    uint8_t MultiplexPinIndex; // The pin used to multiplex the temperature sensors
    bool MultiplexEnabled;     // If the temperature sensors are multiplexed

    uint16_t CellsInParallel;                // Number of cells in parallel
    uint16_t CellVoltageLimitLow;            // The minimum voltage of a cell
    uint16_t CellVoltageLimitHigh;           // The maximum voltage of a cell
    uint16_t CellVoltageDerateLimitLow;      // The minimum voltage of a cell before derating discharge current linearly
    uint16_t CellVoltageDerateLimitHigh;     // The maximum voltage of a cell before derating charge current linearly
    uint16_t CellTemperatureLimitLow;        // The minimum temperature of a cell
    uint16_t CellTemperatureLimitHigh;       // The maximum temperature of a cell
    uint16_t CellTemperatureDerateLimitLow;  // The minimum temperature of a cell before derating linearly
    uint16_t CellTemperatureDerateLimitHigh; // The maximum temperature of a cell before derating linearly
    uint16_t CellDischargeCurrentLimit;      // The maximum discharge current of a cell in mA
    uint16_t CellChargeCurrentLimit;         // The maximum charge current of a cell in mA
    uint16_t SingleCellCapacity;             // The capacity of a single cell in mAh

    uint16_t FuseCurrentLimit;      // The fuse current limit for the BMS, this is the maximum current that can flow through the fuse
    uint16_t DischargeCurrentLimit; // The maximum discharge current of the BMS in mA
    uint16_t ChargeCurrentLimit;    // The maximum charge current of the BMS in mA

    uint8_t CanNodeID;                    // This follows the CAN ID format specified by DTI
    uint8_t CanConfigNodeID;              // Node ID for the configuration CAN messages, back and forth communication with the BMS
    BMS_Config_CanSpeedTypeDef CanSpeed;  // The baudrate of the CAN bus
    uint8_t BroadcastPacketID;            // The Packet ID of the first broadcast can_id. Consecutive packets will have this ID + 1, + 2, etc.
    uint16_t CanBroadcastInterval;        // How often should the BMS broadcast general information
    uint16_t CanTempBroadcastInterval;    // How often should the BMS broadcast temperature information
    uint16_t CanVoltageBroadcastInterval; // How often should the BMS broadcast voltage information
    uint16_t CanChargerBroadcastInterval; // How often should the BMS broadcast the charger information
    uint16_t CanChargerBroadcastTimeout;  // How long should the BMS wait for a charger broadcast before considering it timed out
    bool CanExtended;                     // Should the CAN ID be extended or not

    char MemoryCheck[5]; // Inital check of config, should default to "align", placed last to verify alignment
    uint32_t Checksum;   // The checksum of the config

} BMS_Config_HandleTypeDef;

typedef enum
{
    BMS_CONFIG_PARAM_NUM_OF_SLAVES = 0,                       // uint8_t NumOfSlaves
    BMS_CONFIG_PARAM_CELLS_EACH = 1,                          // uint8_t CellsEach
    BMS_CONFIG_PARAM_TEMPS_EACH = 2,                          // uint8_t TempsEach
    BMS_CONFIG_PARAM_FIRST_TEMP_PIN_INDEX = 3,                // uint8_t FirstTempPinIndex
    BMS_CONFIG_PARAM_MULTIPLEX_PIN_INDEX = 4,                 // uint8_t MultiplexPinIndex
    BMS_CONFIG_PARAM_MULTIPLEX_ENABLED = 5,                   // bool MultiplexEnabled
    BMS_CONFIG_PARAM_CELLS_IN_PARALLEL = 6,                   // uint16_t CellsInParallel
    BMS_CONFIG_PARAM_CELL_VOLTAGE_LIMIT_LOW = 7,              // uint16_t CellVoltageLimitLow
    BMS_CONFIG_PARAM_CELL_VOLTAGE_LIMIT_HIGH = 8,             // uint16_t CellVoltageLimitHigh
    BMS_CONFIG_PARAM_CELL_VOLTAGE_DERATE_LIMIT_LOW = 9,       // uint16_t CellVoltageDerateLimitLow
    BMS_CONFIG_PARAM_CELL_VOLTAGE_DERATE_LIMIT_HIGH = 10,     // uint16_t CellVoltageDerateLimitHigh
    BMS_CONFIG_PARAM_CELL_TEMPERATURE_LIMIT_LOW = 11,         // uint16_t CellTemperatureLimitLow
    BMS_CONFIG_PARAM_CELL_TEMPERATURE_LIMIT_HIGH = 12,        // uint16_t CellTemperatureLimitHigh
    BMS_CONFIG_PARAM_CELL_TEMPERATURE_DERATE_LIMIT_LOW = 13,  // uint16_t CellTemperatureDerateLimitLow
    BMS_CONFIG_PARAM_CELL_TEMPERATURE_DERATE_LIMIT_HIGH = 14, // uint16_t CellTemperatureDerateLimitHigh
    BMS_CONFIG_PARAM_CELL_DISCHARGE_CURRENT_LIMIT = 15,       // uint16_t CellDischargeCurrentLimit
    BMS_CONFIG_PARAM_CELL_CHARGE_CURRENT_LIMIT = 16,          // uint16_t CellChargeCurrentLimit
    BMS_CONFIG_PARAM_SINGLE_CELL_CAPACITY = 17,               // uint16_t SingleCellCapacity
    BMS_CONFIG_PARAM_FUSE_CURRENT_LIMIT = 18,                 // uint16_t FuseCurrentLimit
    BMS_CONFIG_PARAM_DISCHARGE_CURRENT_LIMIT = 19,            // uint16_t DischargeCurrentLimit
    BMS_CONFIG_PARAM_CHARGE_CURRENT_LIMIT = 20,               // uint16_t ChargeCurrentLimit
    BMS_CONFIG_PARAM_CAN_NODE_ID = 21,                        // uint8_t CanNodeID
    BMS_CONFIG_PARAM_CAN_CONFIG_NODE_ID = 22,                 // uint8_t CanConfigNodeID
    BMS_CONFIG_PARAM_CAN_SPEED = 23,                          // BMS_Config_CanSpeedTypeDef CanSpeed
    BMS_CONFIG_PARAM_BROADCAST_PACKET_ID = 24,                // uint8_t BroadcastPacketID
    BMS_CONFIG_PARAM_CAN_BROADCAST_INTERVAL = 25,             // uint16_t CanBroadcastInterval
    BMS_CONFIG_PARAM_CAN_TEMP_BROADCAST_INTERVAL = 26,        // uint16_t CanTempBroadcastInterval
    BMS_CONFIG_PARAM_CAN_VOLTAGE_BROADCAST_INTERVAL = 27,     // uint16_t CanVoltageBroadcastInterval
    BMS_CONFIG_PARAM_CAN_CHARGER_BROADCAST_INTERVAL = 28,     // uint16_t CanChargerBroadcastInterval
    BMS_CONFIG_PARAM_CAN_CHARGER_BROADCAST_TIMEOUT = 29,      // uint16_t CanChargerBroadcastTimeout
    BMS_CONFIG_PARAM_CAN_EXTENDED = 30                        // bool CanExtended

} BMS_Config_ParameterIndexTypeDef;

typedef enum
{
    BMS_CONFIG_OK,
    BMS_CONFIG_ERROR,
    BMS_CONFIG_TIMEOUT,
    BMS_CONFIG_INVALID_CONFIG,
    BMS_CONFIG_INVALID_VALUE,
    BMS_CONFIG_INVALID_PACKET,
} BMS_Config_StatusTypeDef;

void BMS_Config_Init(BMS_Config_HandleTypeDef *bms_config); // This cannot really fail, so it does not return anything

void BMS_Config_SetParameter(BMS_Config_HandleTypeDef *bms_config, BMS_Config_ParameterIndexTypeDef index, uint16_t value); // Set a parameter in the configuration, index is the parameter index, value is the value to set

BMS_Config_StatusTypeDef BMS_Config_WriteToFlash(BMS_Config_HandleTypeDef *bms_config);
BMS_Config_StatusTypeDef BMS_Config_UpdateFromFlash(BMS_Config_HandleTypeDef *bms_config);
BMS_Config_StatusTypeDef BMS_Config_HandleCanMessage(BMS_Config_HandleTypeDef *bms_config, uint16_t packet_id, uint8_t *can_data);

#endif // __BMS_CONFIG_H