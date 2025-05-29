#ifndef FAULTS_H
#define FAULTS_H

#include "stdint.h"

typedef enum
{
  BMS_OK = 0,
  BMS_FAULT_BQ = (1 << 0),                // These are considered fatal errors, and the system will not run
  BMS_FAULT_ADC = (1 << 1),               // These are considered fatal errors, and the system will not run
  BMS_WARNING_OVERCURRENT = (1 << 2),     // This will make the BMS stop the car
  BMS_WARNING_UNDERVOLTAGE = (1 << 3),    // This will make the BMS stop the car
  BMS_WARNING_OVERTEMPERATURE = (1 << 4), // This will make the BMS stop the car
  BMS_WARNING_INT_COMM = (1 << 5),        // This will make the BMS stop the car
  BMS_WARNING_CAN = (1 << 6),             // CAN is not present or not working, This will make the BMS stop the car
  BMS_NOTE_EEPROM = (1 << 7),             // EEPROM is not present or not working, but the system is using a config from RAM, and operating normally
} BMS_FaultTypeDef;

#define BMS_FAULT_MASK ((BMS_FaultFlags)(BMS_FAULT_BQ | BMS_FAULT_ADC))
#define BMS_WARNING_MASK ((BMS_FaultFlags) (BMS_WARNING_CAN | BMS_WARNING_OVERCURRENT | BMS_WARNING_UNDERVOLTAGE | BMS_WARNING_OVERTEMPERATURE | BMS_WARNING_INT_COMM))

typedef uint8_t BMS_FaultFlags; // This is a bitwise select of the faults and warnings

#endif // FAULTS_H