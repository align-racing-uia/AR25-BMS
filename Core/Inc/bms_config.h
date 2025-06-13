#ifndef __BMS_CONFIG_H
#define __BMS_CONFIG_H

#include "stdint.h"
#include "stddef.h"
#include "stdbool.h"

#define BMS_CONFIG_VERSION 2 // This number will automatically increment when the config changes

// These are absolute maxes for the battery model, not the actual values
// The actual values are set in the battery model init function

#define TEMP_MAP_POOL_MAX_POINTS 100 // Size of the OCV map pool, defaults to a maximum of 5 temperature maps with 15 points each
#define TEMP_MAP_POOL_AMOUNT 5 // Size of the temperature map pool, defaults to a maximum of 5 temperature maps with 15 points each

#define BQ_MAX_AMOUNT_OF_CHIPS 15 // The maximum amount of chips in the system
#define BQ_MAX_AMOUNT_OF_SLAVES (BQ_MAX_AMOUNT_OF_CHIPS-1) // The maximum amount of BQ79616 chips in the system
#define BQ_MAX_AMOUNT_OF_CELLS_EACH 16 // The maximum amount of cells in series on each board
#define BQ_MAX_AMOUNT_OF_TEMPS_EACH 14 // The maximum amount of temperature sensors on each board

#define CELL_MEMORY_POOL_SIZE (BQ_MAX_AMOUNT_OF_SLAVES * BQ_MAX_AMOUNT_OF_CELLS_EACH)  // Size of the cell memory pool, defaults to a maximum of 300 cells

// Default values for compiled programs can be set in this header
// These values are only used if the EEPROM is empty, corrupt, or not present
#define DEFAULT_TOTAL_CHIPS 3       // Including master
#define DEFAULT_TOTAL_SLAVES (DEFAULT_TOTAL_CHIPS-1)     // Number of slaves in the system
#define DEFAULT_CELLS_EACH 14       // Number of cells in series on each slave
#define DEFAULT_TEMPS_EACH 6       // Number of temperature sensors on each slave
#define DEFAULT_TEMP_MAP_VOLTAGE_POINTS 5 // Number of voltage points in each temperature map
#define DEFAULT_TEMP_MAP_AMOUNT 3 // Number of temperature maps
#define DEFAULT_CELLS_IN_PARALLEL 1 // Number of cells in parallel
#define DEFAULT_CELLVOLTAGE_LIMIT_LOW 2000   // mV
#define DEFAULT_CELLVOLTAGE_LIMIT_HIGH 4200  // mV
#define DEFAULT_CELLTEMPERATURE_LIMIT_LOW 0 // C
#define DEFAULT_CELLTEMPERATURE_LIMIT_HIGH 60 // C
#define DEFAULT_CAN_NODE_ID 0x06
#define DEFAULT_CAN_CONFIG_NODE_ID (DEFAULT_CAN_NODE_ID+1) // The CAN ID of the BMS, this is used to identify the BMS on the CAN bus
#define DEFAULT_CAN_BROADCAST_PACKET 0x01
#define DEFAULT_CAN_BAUDRATE (uint16_t) 500000 // 500kbit/s
#define DEFAULT_CAN_EXTENDED false // Should the CAN ID be extended or not
#define DEFAULT_BROADCAST_PACKET 0x01
#define DEFAULT_USB_LOGGING_ENABLED 0 // Should the USB logging be enabled or not
#define DEFAULT_USB_LOGGING_INTERVAL 500 // The interval for the USB logging
#define DEFAULT_CAN_BROADCAST_INTERVAL 100 // The interval for the CAN broadcast
#define DEFAULT_CAN_TEMP_BROADCAST_INTERVAL 1000 // The interval for the CAN temperature broadcast
#define DEFAULT_CAN_CHARGER_BROADCAST_INTERVAL 1000 // The interval for the CAN charger broadcast
#define DEFAULT_CAN_CHARGER_BROADCAST_TIMEOUT 5000 // The timeout for the charger broadcast packet  
#define DEFAULT_BALANCE_WHILE_CHARGING false // Should the BMS balance while charging or not

#define DEFAULT_MULTIPLEX_ENABLED true // If the temperature sensors are multiplexed
#define DEFAULT_MULTIPLEX_PIN_INDEX 7 // The pin used to multiplex the temperature sensors

#define DEFAULT_TOTAL_CELLS (DEFAULT_TOTAL_SLAVES * DEFAULT_CELLS_EACH * DEFAULT_CELLS_IN_PARALLEL)
#define DEFAULT_TOTAL_CELLS_IN_SERIES (DEFAULT_TOTAL_SLAVES * DEFAULT_CELLS_EACH)

// All of these parameters can be set through serial, and will be stored on the flash
typedef struct
{
    uint16_t ConfigVersion;            // This number will automatically increment when the config changes
    uint8_t NumOfChips;                // The number of chips in the system
    uint16_t CellCount;                // Total number of cells
    uint8_t  NumOfSlaves;              // The number of slaves in the system
    uint8_t  CellsEach;
    uint8_t  TempsEach;
    uint16_t TotalCellCountInSeries;   // Total number of cells in series
    uint16_t CellCountInParallel;      // Number of cells in parallel
    uint16_t CellVoltageLimitLow;      // The minimum voltage of a cell
    uint16_t CellVoltageLimitHigh;     // The maximum voltage of a cell
    uint16_t CellTemperatureLimitLow;  // The minimum temperature of a cell
    uint16_t CellTemperatureLimitHigh; // The maximum temperature of a cell
    
    char MemoryCheck[5];               // Inital check of config, should default to "align"
    uint16_t CCWarningLimit; // The charging current limit for the BMS during warning state
    uint16_t DCWarningLimit; // The discharge current limit for the BMS during warning state

    uint8_t TempMapAmount; // The number of temperature maps
    uint8_t TempMapVoltagePoints; // The number of voltage points in each temperature map

    uint8_t CanNodeID;       // This follows the CAN ID format specified by DTI
    uint8_t CanConfigNodeID;       // This follows the CAN ID format specified by DTI
    uint16_t CanBaudrate;    // The baudrate of the CAN bus
    bool CanExtended; // Should the CAN ID be extended or not
    bool UsbLoggingEnabled; // Should the USB logging be enabled or not
    bool CanVoltageBroadcastEnabled;
    bool CanTempBroadcastEnabled;

    uint8_t BroadcastPacket; // The ID of the board

    uint16_t CanBroadcastInterval; // The timeout for the broadcast packet
    uint16_t CanTempBroadcastInterval; // The timeout for the temperature broadcast packet
    uint16_t CanVoltageBroadcastInterval; // The timeout for the temperature broadcast packet
    uint16_t UsbLoggingInterval; // The timeout for the USB logging
    uint16_t CanChargerBroadcastInterval; // The timeout for the charger broadcast packet
    uint16_t CanChargerBroadcastTimeout; // The timeout for the charger broadcast packet
    bool BalanceWhileCharging; // Should the BMS balance while charging or not

    uint32_t Checksum;       // The checksum of the config

} BMS_Config_HandleTypeDef;

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

void BMS_Config_SetParameter(BMS_Config_HandleTypeDef *bms_config, uint8_t index, uint16_t value); // Set a parameter in the configuration, index is the parameter index, value is the value to set

BMS_Config_StatusTypeDef BMS_Config_WriteToFlash(BMS_Config_HandleTypeDef *bms_config);
BMS_Config_StatusTypeDef BMS_Config_UpdateFromFlash(BMS_Config_HandleTypeDef *bms_config);
BMS_Config_StatusTypeDef BMS_Config_HandleCanMessage(BMS_Config_HandleTypeDef *bms_config, uint16_t packet_id, uint8_t *can_data);

#endif // __BMS_CONFIG_H