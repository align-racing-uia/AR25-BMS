#ifndef FAULTS_H
#define FAULTS_H

#include "stdint.h"

// Faults will stop the BMS from running normally, and will require a power cycle (and repairs) to reset
typedef enum
{
  BMS_FAULT_NONE = 0,
  BMS_FAULT_BQ_NOT_CONNECTED = (1 << 0),                // These are considered fatal errors, and the system will not run
  BMS_FAULT_CRITICAL_TEMPERATURE = (1 << 1),               // These are considered fatal errors, and the system will not run
  BMS_FAULT_LOST_TEMPERATURE_SENSOR = (1 << 2),               // These are considered fatal errors, and the system will not run
  BMS_FAULT_CRITICAL_VOLTAGE = (1 << 3),               // These are considered fatal errors, and the system will not run
} BMS_FaultTypeDef;

// Warnings will stop the Tractive System from running, but will not stop the BMS from running
typedef enum
{
  BMS_WARNING_NONE = 0,
  BMS_WARNING_OVERCURRENT = (1 << 0),    
  BMS_WARNING_UNDERVOLTAGE = (1 << 1),   
  BMS_WARNING_OVERTEMPERATURE = (1 << 2),
  BMS_WARNING_CAN = (1 << 3),
  BMS_WARNING_NO_EEPROM = (1 << 4), 
} BMS_WarningTypeDef;


#endif // FAULTS_H