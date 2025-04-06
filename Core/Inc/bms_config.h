#ifndef __BMS_CONFIG_H
#define __BMS_CONFIG_H

#include "stdint.h"
#include "stdbool.h"

// Default values for compiled programs can be set in this header
#define DEFAULT_TOTALBOARDS 2       // Including base
#define DEFAULT_CELLS_IN_SERIES 16  // On each board
#define DEFAULT_CELLS_IN_PARALLEL 1 // Number of cells in parallel
#define DEFAULT_CELLVOLTAGE_LIMIT_LOW 2000   // mV
#define DEFAULT_CELLVOLTAGE_LIMIT_HIGH 4200  // mV
#define DEFAULT_CELLTEMPERATURE_LIMIT_LOW 0 // C
#define DEFAULT_CELLTEMPERATURE_LIMIT_HIGH 60 // C
#define DEFAULT_CAN_NODE_ID 0x01
#define DEFAULT_CAN_BROADCAST_PACKET 0x01
#define DEFAULT_CAN_BAUDRATE (uint16_t) 500000 // 500kbit/s
#define DEFAULT_CAN_EXTENDED 0 // Should the CAN ID be extended or not
#define DEFAULT_BROADCAST_PACKET 0x01

#define DEFAULT_TOTAL_CELLS (DEFAULT_TOTALBOARDS * DEFAULT_CELLS_IN_SERIES) // Counts cells in parallel as one

// All of these parameters can be set through serial, and will be stored on the flash
typedef struct
{
    uint16_t ConfigVersion;            // This number will automatically increment when the config changes
    char MemoryCheck[5];               // Inital check of config, should default to "align"
    uint8_t NumOfBoards;                // The number of boards in the system, including the base
    uint16_t CellCount;                // Total number of cells
    // TODO: Make sure that the voltage readings are based on this
    uint8_t  ChipCount;              // The number of chips in the system, not including the base 
    uint16_t TemperatureSensorCount;        // Total number of temperature sensors for each BQ79616
    uint16_t CellCountInSeries;        // Number of cells in series
    uint16_t CellCountInParallel;      // Number of cells in parallel
    uint16_t CellVoltageLimitLow;      // The minimum voltage of a cell
    uint16_t CellVoltageLimitHigh;     // The maximum voltage of a cell
    uint16_t CellTemperatureLimitLow;  // The minimum temperature of a cell
    uint16_t CellTemperatureLimitHigh; // The maximum temperature of a cell

    uint8_t CanNodeID;       // This follows the CAN ID format specified by DTI
    uint16_t CanBaudrate;    // The baudrate of the CAN bus
    bool CanExtended; // Should the CAN ID be extended or not
    uint8_t BroadcastPacket; // The ID of the board
    uint32_t Checksum;       // The checksum of the config

} BMS_ConfigTypeDef;

void BMS_Config_WriteToFlash(BMS_ConfigTypeDef *bms_config, uint8_t* data, uint16_t size);
void BMS_Config_ReadFromFlash(BMS_ConfigTypeDef *bms_config, uint8_t* data, uint16_t size);


#endif // __BMS_CONFIG_H