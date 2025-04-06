#ifndef BATTERY_MODEL_H
#define BATTERY_MODEL_H

#include <stdint.h>
#include "bq79600.h"
#include "bms_config.h"

typedef struct {
    
    uint16_t CellID; // No battery in align until this point has had more than 255 cells
    uint16_t TemperatureID; // What sensor to listen to for temperature
    float EstimatedVoltage;
    float EstimatedCurrent;
    float MeasuredVoltage;
    float MeasuredTemperature;

} CellModel_HandleTypeDef;

typedef struct {

    int8_t AverageTemperature;
    uint8_t PackVoltage;
    int16_t PackCurrent; 
    uint8_t CellCount;
    CellModel_HandleTypeDef *Cells;

} BatteryModel_HandleTypeDef;

void BatteryModel_Init(BatteryModel_HandleTypeDef *battery_model, uint16_t cell_count);
void BatteryModel_UpdateMeasured(BatteryModel_HandleTypeDef *battery_model, float *cell_voltages, float *die_temperatures);


#endif // BATTERY_MODEL_H