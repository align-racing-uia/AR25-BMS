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
    BMS_STATE_BOOTING = 0, // Initial state, the BMS is initializing
    BMS_STATE_CONNECTING,
    BMS_STATE_CONFIGURING,
    BMS_STATE_IDLE,
    BMS_STATE_ACTVATING_TS,
    BMS_STATE_CHARGING,
    BMS_STATE_DRIVING,
    BMS_STATE_FAULT
} BMS_StateTypeDef;

typedef struct
{
    BMS_StateTypeDef State;                  // The state of the BMS
    BMS_FaultFlags ActiveFaults;             // Active faults bitmask
    BatteryModel_HandleTypeDef* BatteryModel; // Battery model handle
    TS_HandleTypeDef* TS;                     // Tractive system state machine handle
    BQ_HandleTypeDef* BQ;                     // BQ79600 handle
    BMS_Config_HandleTypeDef* Config;         // BMS configuration handle
    
    bool WarningPresent;                     // Warning present flag
    bool FaultPresent;                       // Fault present flag
    bool SDC;                                // SDC connected flag

} BMS_HandleTypeDef;

void BMS_BindMemory(BMS_HandleTypeDef *hbms, BatteryModel_HandleTypeDef *battery_model, BQ_HandleTypeDef *bq);      
void BMS_Update(BMS_HandleTypeDef *hbms);

#endif // BMS_STATEMACHINE_H