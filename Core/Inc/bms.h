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

typedef struct 
{
    GPIO_TypeDef *Port; // GPIO port
    uint16_t Pin;       // GPIO pin number
} BMS_PinTypeDef;

typedef struct
{

    FDCAN_HandleTypeDef *hfdcan; // Handle for the FDCAN peripheral
    BMS_PinTypeDef FaultPin;      // Pin for the fault indicator

} BMS_HardwareConfigTypeDef;

typedef struct
{
    BatteryModel_HandleTypeDef *BatteryModel; // Battery model handle
    TS_HandleTypeDef *TS;                     // Tractive system state machine handle
    BQ_HandleTypeDef *BQ;                     // BQ79600 handle
    FDCAN_HandleTypeDef *FDCAN;              // Handle for the FDCAN peripheral

    BMS_Config_HandleTypeDef Config; // BMS configuration handle

    BMS_StateTypeDef State;      // The state of the BMS
    BMS_FaultFlags ActiveFaults; // Active faults bitmask
    
    BMS_PinTypeDef FaultPin;     // Pin for the fault indicator

    uint32_t CanTimestamp;     // Timestamp for the last CAN message
    uint32_t ChargerTimestamp; // Timestamp for the last charger CAN message
    
    bool WarningPresent; // Warning present flag
    bool EepromPresent;  // EEPROM present flag
    bool ChargerPresent; // Charger connected flag

    bool SdcClosed;   // SdcClosed connected flag
    bool Initialized; // Initialized flag, true if the BMS is initialized

} BMS_HandleTypeDef;

void BMS_Init(BMS_HandleTypeDef *hbms, BMS_HardwareConfigTypeDef *hardware_config);
void BMS_BindMemory(BMS_HandleTypeDef *hbms, BatteryModel_HandleTypeDef *battery_model, BQ_HandleTypeDef *bq);
void BMS_Update(BMS_HandleTypeDef *hbms);

#endif // BMS_STATEMACHINE_H