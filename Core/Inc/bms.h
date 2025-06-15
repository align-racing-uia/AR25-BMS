#ifndef BMS_STATEMACHINE_H
#define BMS_STATEMACHINE_H

#include "faults.h"
#include "stdbool.h"
#include "battery_model.h"
#include "bq79600.h"
#include "bms_config.h"

typedef enum
{
    BMS_STATE_CONFIGURING, // Initial state, loading configuration
    BMS_STATE_CONNECTING,
    BMS_STATE_IDLE,
    BMS_STATE_CHARGING,
    BMS_STATE_TS_ACTIVE,
    BMS_STATE_FAULT
} BMS_StateTypeDef;

typedef enum
{
  TS_STATE_START = 0,
  TS_STATE_PRECHARGE = 1,
  TS_STATE_READY = 2,
  TS_STATE_STOPPED = 3,
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

    // Pins for the AIRs
    BMS_PinTypeDef PlusAIR; // Pin for the high current sensor
    BMS_PinTypeDef MinusAIR; // Pin for the high current sensor
    BMS_PinTypeDef PrechargeAIR; // Pin for the high current sensor
    BMS_PinTypeDef SdcPin; // Pin for the SdcClosed indicator
    
} BMS_HardwareConfigTypeDef;

typedef struct
{
    BatteryModel_HandleTypeDef *BatteryModel; // Battery model handle
    BQ_HandleTypeDef *BQ;                     // BQ79600 handle
    FDCAN_HandleTypeDef *FDCAN;               // Handle for the FDCAN peripheral

    BMS_Config_HandleTypeDef Config; // BMS configuration handle

    BMS_StateTypeDef State;      // The state of the BMS
    BMS_TS_StateTypeDef TSState; // The state of the TS
    uint8_t ActiveFaults; // Active faults bitmask
    uint8_t ActiveWarnings; // Active warnings bitmask

    BMS_PinTypeDef FaultPin;             // Pin for the fault indicator
    BMS_PinTypeDef LowCurrentSensorPin;  // Pin for the low current sensor
    BMS_PinTypeDef HighCurrentSensorPin; // Pin for the high current sensor

    // Pins for the AIRs
    BMS_PinTypeDef PlusAIR; // Pin for the high current sensor
    BMS_PinTypeDef MinusAIR; // Pin for the high current sensor
    BMS_PinTypeDef PrechargeAIR; // Pin for the high current sensor
    BMS_PinTypeDef SdcPin; // Pin for the SdcClosed indicator

    // State variables related to the Tractive System (TS)
    bool SdcClosed;   // SdcClosed connected flag
    bool TsRequested; // TS requested flag, true if the TS is requested to be active
    bool ChargerPresent; // Charger connected flag

    uint32_t CanTimestamp;     // Timestamp for the last CAN message
    uint32_t ChargerPresentTimestamp; // Timestamp for the last charger CAN message
    uint32_t ChargerBroadcastTimestamp;
    uint32_t TempTimestamp;    // Timestamp for the last temperature measurement
    uint32_t VoltageTimestamp; // Timestamp for the last voltage measurement

    float MeasuredCurrent; // Measured current from the sensors

    // Exposed state variables, from other handles
    float *HighestCellTemperature; // Highest cell temperature in the pack
    float *LowestCellTemperature;  // Lowest cell temperature in the pack
    float *HighestCellVoltage;     // Highest cell voltage in the pack
    float *LowestCellVoltage;      // Lowest cell voltage in the pack
    float *CellVoltages;           // Array of cell voltages
    float *CellTemperatures;       // Array of cell temperatures
    float *PackVoltage;            // Pack voltage

    // Paramterers relevant for the BMS fetched over CAN
    uint16_t InverterVoltage; //


    // Toggles for broadcasts
    bool BroadcastVoltages; // Flag to indicate if the voltages should be broadcasted
    bool BroadcastTemperatures; // Flag to indicate if the temperatures should be broadcasted


    uint16_t DcLimit; // Discharge current limit in A x 10
    uint16_t CcLimit; // Charge current limit in A x 10
    
    float *SOC; // State of charge in percentage
    bool WarningPresent; // Warning present flag
    bool EepromPresent;  // EEPROM present flag
    bool BqConnected;    // BQ connected flag

    uint32_t LastMeasurementTimestamp;
    uint32_t BroadcastTimestamp; // Timestamp for the last broadcast
    uint32_t PrechargeTimestamp; // Timestamp for the last precharge

    bool Initialized; // Initialized flag, true if the BMS is initialized

} BMS_HandleTypeDef;

void BMS_Init(BMS_HandleTypeDef *hbms, BMS_HardwareConfigTypeDef *hardware_config);
void BMS_BindMemory(BMS_HandleTypeDef *hbms, BatteryModel_HandleTypeDef *battery_model, BQ_HandleTypeDef *bq);
void BMS_Update(BMS_HandleTypeDef *hbms);

#endif // BMS_STATEMACHINE_H