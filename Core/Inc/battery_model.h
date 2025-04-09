#ifndef BATTERY_MODEL_H
#define BATTERY_MODEL_H

#include <stdint.h>
#include "bq79600.h"
#include "bms_config.h"

// A battery model implemented based on the model used in the following paper:
// https://www.sciencedirect.com/science/article/pii/S0360544211002271#sec2




typedef struct {
    
    uint16_t CellID; // No battery in align until this point has had more than 255 cells
    uint16_t TemperatureID; // What sensor to listen to for temperature
    float EstimatedVoltage;
    float EstimatedVoltageError;
    float EstimatedSOC;
    float EstimatedCapacity;
    float SpentEnergy;
    float MeasuredVoltage;
    float MeasuredRestingVoltage; // Last measured voltage with no current draw
    float MeasuredTemperature;
    float MeasuredCurrent;
    float MeasuredResistance;
    float NominalCapacity; // Nominal capacity of the cell
    float NominalVoltage; // Nominal voltage of the cell
} CellModel_HandleTypeDef;


typedef struct {
    uint16_t Size;
    uint8_t Temperature;
    float *VoltagePoints; // Voltage points for certain SOCs
} TempMap_HandleTypeDef;

typedef struct {
    uint16_t Size; // The number 0-100 SOC should be divided on, saves a bit of memory with the tradeof that you have to do a bit of math
    uint16_t NumOfMaps; // Number of maps
    TempMap_HandleTypeDef **TemperatureMaps; // This contains voltage points for a certain temperature
    float *SOCPoints; // This contains SOC points to be matched with the voltages
} OCV_HandleTypeDef;


typedef struct {

    float AverageTemperature;
    uint16_t PackVoltage;
    int16_t PackCurrent; 
    uint16_t CellCount;
    uint16_t CellsInSeries;
    uint16_t CellsInParallel;
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


void BatteryModel_Init(BatteryModel_HandleTypeDef *battery_model, CellModel_HandleTypeDef* cell_memory_pool, uint16_t cell_count, uint16_t cells_in_series, uint16_t cells_in_parallel);
void BatteryModel_LoadCellData(BatteryModel_HandleTypeDef *battery_model, float k0, float k1, float k2, float k3, float k4, float discharge_resistance, float charging_resistance, float hysteresis);
void BatteryModel_UpdateMeasured(BatteryModel_HandleTypeDef *battery_model, float *cell_voltages, float *cell_temperatures, float *total_current);
void BatteryModel_UpdateEstimates(BatteryModel_HandleTypeDef *battery_model);
void BatteryModel_InitOCVMaps(BatteryModel_HandleTypeDef *battery_model, uint16_t amount_of_voltage_points, float* voltage_point_memory_pool, float* soc_point_memory_pool, TempMap_HandleTypeDef* temp_map_memory_pool, uint8_t numOfMaps);


#endif // BATTERY_MODEL_H