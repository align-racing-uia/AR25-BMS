#ifndef __BMS_CONFIG_H
#define __BMS_CONFIG_H

#include "stdint.h"
#include "stddef.h"
#include "stdbool.h"

#define BMS_CONFIG_VERSION 2 // This number will automatically increment when the config changes

// These are absolute maxes for the battery model, not the actual values
// The actual values are set in the battery model init function

#define TEMP_MAP_POOL_MAX_POINTS 100 // Size of the OCV map pool, defaults to a maximum of 5 temperature maps with 15 points each
#define TEMP_MAP_POOL_AMOUNT 5       // Size of the temperature map pool, defaults to a maximum of 5 temperature maps with 15 points each

#define BQ_MAX_AMOUNT_OF_CHIPS 15                            // The maximum amount of chips in the system
#define BQ_MAX_AMOUNT_OF_SLAVES (BQ_MAX_AMOUNT_OF_CHIPS - 1) // The maximum amount of BQ79616 chips in the system
#define BQ_MAX_AMOUNT_OF_CELLS_EACH 16                       // The maximum amount of cells in series on each board
#define BQ_MAX_AMOUNT_OF_TEMPS_EACH 14                       // The maximum amount of temperature sensors on each board

#define CELL_MEMORY_POOL_SIZE (BQ_MAX_AMOUNT_OF_SLAVES * BQ_MAX_AMOUNT_OF_CELLS_EACH) // Size of the cell memory pool, defaults to a maximum of 300 cells

// Default values for compiled programs can be set in this header
// These values are only used if the EEPROM is empty, corrupt, or not present
#define DEFAULT_TOTAL_CHIPS 11                         // Including master
#define DEFAULT_TOTAL_SLAVES (DEFAULT_TOTAL_CHIPS - 1) // Number of slaves in the system
#define DEFAULT_CELLS_EACH 14                          // Number of cells in series on each slave
#define DEFAULT_TEMPS_EACH 6                           // Number of temperature sensors on each slave
#define DEFAULT_TEMP_MAP_VOLTAGE_POINTS 5              // Number of voltage points in each temperature map
#define DEFAULT_TEMP_MAP_AMOUNT 3                      // Number of temperature maps
#define DEFAULT_CELLS_IN_PARALLEL 1                    // Number of cells in parallel
#define DEFAULT_CELLVOLTAGE_LIMIT_LOW 2000             // mV
#define DEFAULT_CELLVOLTAGE_LIMIT_HIGH 4200            // mV
#define DEFAULT_CELLTEMPERATURE_LIMIT_LOW 0            // C
#define DEFAULT_CELLTEMPERATURE_LIMIT_HIGH 60          // C
#define DEFAULT_CAN_NODE_ID 0x06
#define DEFAULT_CAN_CONFIG_NODE_ID (DEFAULT_CAN_NODE_ID + 1) // The CAN ID of the BMS, this is used to identify the BMS on the CAN bus
#define DEFAULT_CAN_BROADCAST_PACKET 0x01
#define DEFAULT_CAN_BAUDRATE (uint16_t)500000 // 500kbit/s
#define DEFAULT_CAN_EXTENDED false            // Should the CAN ID be extended or not
#define DEFAULT_BROADCAST_PACKET 0x01
#define DEFAULT_USB_LOGGING_ENABLED 0               // Should the USB logging be enabled or not
#define DEFAULT_USB_LOGGING_INTERVAL 500            // The interval for the USB logging
#define DEFAULT_CAN_BROADCAST_INTERVAL 100          // The interval for the CAN broadcast
#define DEFAULT_CAN_TEMP_BROADCAST_INTERVAL 1000    // The interval for the CAN temperature broadcast
#define DEFAULT_CAN_CHARGER_BROADCAST_INTERVAL 1000 // The interval for the CAN charger broadcast
#define DEFAULT_CAN_CHARGER_BROADCAST_TIMEOUT 5000  // The timeout for the charger broadcast packet
#define DEFAULT_BALANCE_WHILE_CHARGING false        // Should the BMS balance while charging or not

#define DEFAULT_MULTIPLEX_ENABLED true // If the temperature sensors are multiplexed
#define DEFAULT_MULTIPLEX_PIN_INDEX 7  // The pin used to multiplex the temperature sensors

#define DEFAULT_TOTAL_CELLS (DEFAULT_TOTAL_SLAVES * DEFAULT_CELLS_EACH * DEFAULT_CELLS_IN_PARALLEL)
#define DEFAULT_TOTAL_CELLS_IN_SERIES (DEFAULT_TOTAL_SLAVES * DEFAULT_CELLS_EACH)

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

    uint16_t FuseCurrentLimit; // The fuse current limit for the BMS, this is the maximum current that can flow through the fuse
    uint16_t CCWarningLimit;   // The charging current limit for the BMS during warning state (No AMS fault, only open AIRs)
    uint16_t DCWarningLimit;   // The discharge current limit for the BMS during warning state (No AMS fault, only open AIRs)

    uint8_t CanNodeID;                    // This follows the CAN ID format specified by DTI
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
    BMS_CONFIG_PARAM_CC_WARNING_LIMIT = 19,                   // uint16_t CCWarningLimit
    BMS_CONFIG_PARAM_DC_WARNING_LIMIT = 20,                   // uint16_t DCWarningLimit
    BMS_CONFIG_PARAM_CAN_NODE_ID = 21,                        // uint8_t CanNodeID
    BMS_CONFIG_PARAM_CAN_SPEED = 22,                          // BMS_Config_CanSpeedTypeDef CanSpeed
    BMS_CONFIG_PARAM_BROADCAST_PACKET_ID = 23,                // uint8_t BroadcastPacketID
    BMS_CONFIG_PARAM_CAN_BROADCAST_INTERVAL = 24,             // uint16_t CanBroadcastInterval
    BMS_CONFIG_PARAM_CAN_TEMP_BROADCAST_INTERVAL = 25,        // uint16_t CanTempBroadcastInterval
    BMS_CONFIG_PARAM_CAN_VOLTAGE_BROADCAST_INTERVAL = 26,     // uint16_t CanVoltageBroadcastInterval
    BMS_CONFIG_PARAM_CAN_CHARGER_BROADCAST_INTERVAL = 27,     // uint16_t CanChargerBroadcastInterval
    BMS_CONFIG_PARAM_CAN_CHARGER_BROADCAST_TIMEOUT = 28,      // uint16_t CanChargerBroadcastTimeout
    BMS_CONFIG_PARAM_CAN_EXTENDED = 29,                       // bool CanExtended
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