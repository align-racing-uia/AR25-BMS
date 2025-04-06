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

    float AverageTemperature;
    uint16_t PackVoltage;
    int16_t PackCurrent; 
    uint16_t CellCount;
    uint16_t CellsInSeries;
    CellModel_HandleTypeDef *Cells;

} BatteryModel_HandleTypeDef;

void BatteryModel_Init(BatteryModel_HandleTypeDef *battery_model, uint16_t cell_count, uint16_t cells_in_series);
void BatteryModel_UpdateMeasured(BatteryModel_HandleTypeDef *battery_model, float *cell_voltages, float *die_temperatures);
void BatteryModel_UpdateEstimates(BatteryModel_HandleTypeDef *battery_model, float *cell_voltages, float *cell_temperatures, float *current_sensor);


#endif // BATTERY_MODEL_H