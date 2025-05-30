#ifndef BATTERY_MODEL_H
#define BATTERY_MODEL_H

#include <stdint.h>
#include "bms_config.h"




// A battery model implemented based on the model used in the following paper:
// https://www.sciencedirect.com/science/article/pii/S0360544211002271#sec2


typedef struct {
    float EstimatedSOC;
    float EstimatedCapacity;
    float SpentEnergy;
    float MeasuredVoltage;
    float MeasuredRestingVoltage; // Last measured voltage with no current draw
    float MeasuredTemperature;
    float MeasuredCurrent;
    float EstimatedResistance;
    float NominalCapacity; // Nominal capacity of the cell
} CellModel_HandleTypeDef;


typedef struct {
    uint8_t Size; // The number 0-100 SOC should be divided on, saves a bit of memory with the tradeof that you have to do a bit of math
    uint8_t Temperature;
    float *VoltagePoints; // Voltage points for certain SOCs
} TempMap_HandleTypeDef;

typedef struct {
    uint16_t NumOfMaps; // Number of maps
    TempMap_HandleTypeDef *TemperatureMaps; // This contains voltage points for a certain temperature
} OCV_HandleTypeDef;


typedef struct {
    bool  soc_estimated; // If the first SOC estimation is done
    float AverageTemperature;
    float EstimatedSOC;
    uint16_t PackVoltage;
    int16_t PackCurrent; 
    uint16_t CellCount;
    uint16_t CellsInSeries;
    uint16_t CellsInParallel;
    uint32_t LastCycle;
    float K0; // Typically the resting voltage
    float K1; // K1 / SOC
    float K2; // K2 * SOC
    float K3; // K3 * ln(SOC)
    float K4; // K4 * ln(1-SOC)
    float DischargeResistance; // Resistance during discharge
    float ChargingResistance; //  Resistance during charging
    float H; // Hysterisis
    CellModel_HandleTypeDef *Cells;
    OCV_HandleTypeDef OCV; // Open circuit voltage map

} BatteryModel_HandleTypeDef;


void BatteryModel_Init(BatteryModel_HandleTypeDef *battery_model, uint16_t cell_count, uint16_t cells_in_series, uint16_t cells_in_parallel, uint16_t nominal_cell_capacity);
void BatteryModel_BindMemory(BatteryModel_HandleTypeDef *battery_model, CellModel_HandleTypeDef* cell_memory_pool);
void BatteryModel_LoadOCVMap(BatteryModel_HandleTypeDef *battery_model, float* voltage_points, uint8_t temp, uint8_t map_index, uint16_t num_of_voltage_points);
void BatteryModel_Update(BatteryModel_HandleTypeDef *battery_model, float *cell_voltages, float *cell_temperatures, float *total_current, uint16_t current_timestamp);

#endif // BATTERY_MODEL_H