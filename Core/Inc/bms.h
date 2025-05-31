#ifndef BMS_STATEMACHINE_H
#define BMS_STATEMACHINE_H

#include "faults.h"
#include "stdbool.h"
#include "battery_model.h"
#include "ts_statemachine.h"
#include "bq79600.h"
#include "bms_config.h"

typedef enum
{
    BMS_STATE_CONFIGURING, // Initial state, loading configuration
    BMS_STATE_CONNECTING,
    BMS_STATE_IDLE,
    BMS_STATE_ACTVATING_TS,
    BMS_STATE_CHARGING,
    BMS_STATE_DRIVING,
    BMS_STATE_FAULT
} BMS_StateTypeDef;

typedef enum
{
  TS_STATE_IDLE = 0,
  TS_STATE_PRECHARGE = 1,
  TS_STATE_ACTIVE = 2,
  TS_STATE_FAULT = 3,
} BMS_TS_StateTypeDef;

typedef struct
{
    GPIO_TypeDef *Port; // GPIO port
    uint16_t Pin;       // GPIO pin number
} BMS_PinTypeDef;

typedef struct
{

    FDCAN_HandleTypeDef *hfdcan;         // Handle for the FDCAN peripheral
    BMS_PinTypeDef FaultPin;             // Pin for the fault indicator
    BMS_PinTypeDef LowCurrentSensorPin;  // Pin for the low current sensor
    BMS_PinTypeDef HighCurrentSensorPin; // Pin for the high current sensor

} BMS_HardwareConfigTypeDef;

typedef struct
{
    BatteryModel_HandleTypeDef *BatteryModel; // Battery model handle
    BQ_HandleTypeDef *BQ;                     // BQ79600 handle
    FDCAN_HandleTypeDef *FDCAN;               // Handle for the FDCAN peripheral

    BMS_Config_HandleTypeDef Config; // BMS configuration handle

    BMS_StateTypeDef State;      // The state of the BMS
    BMS_TS_StateTypeDef TS_State; // The state of the TS
    BMS_FaultFlags ActiveFaults; // Active faults bitmask

    BMS_PinTypeDef FaultPin;             // Pin for the fault indicator
    BMS_PinTypeDef LowCurrentSensorPin;  // Pin for the low current sensor
    BMS_PinTypeDef HighCurrentSensorPin; // Pin for the high current sensor

    uint32_t CanTimestamp;     // Timestamp for the last CAN message
    uint32_t ChargerTimestamp; // Timestamp for the last charger CAN message

    float MeasuredCurrent; // Measured current from the sensors

    // Exposed state variables, from other handles
    float *HighestCellTemperature; // Highest cell temperature in the pack
    float *LowestCellTemperature;  // Lowest cell temperature in the pack
    float *AverageCellVoltage;     // Average cell voltage in the pack
    float *AverageCellTemperature; // Average cell temperature in the pack
    float *PackVoltage;            // Pack voltage
    float *PackCurrent;            // Pack current
    float *HighestCellVoltage;     // Highest cell voltage in the pack
    float *LowestCellVoltage;      // Lowest cell voltage in the pack
    float *CellVoltages;           // Array of cell voltages
    float *CellTemperatures;       // Array of cell temperatures

    bool WarningPresent; // Warning present flag
    bool EepromPresent;  // EEPROM present flag
    bool ChargerPresent; // Charger connected flag
    bool BqConnected;    // BQ connected flag

    bool SdcClosed;   // SdcClosed connected flag
    bool Initialized; // Initialized flag, true if the BMS is initialized

} BMS_HandleTypeDef;

void BMS_Init(BMS_HandleTypeDef *hbms, BMS_HardwareConfigTypeDef *hardware_config);
void BMS_BindMemory(BMS_HandleTypeDef *hbms, BatteryModel_HandleTypeDef *battery_model, BQ_HandleTypeDef *bq);
void BMS_Update(BMS_HandleTypeDef *hbms);

#endif // BMS_STATEMACHINE_H