#include "battery_model.h"
#include "stdlib.h"

void BatteryModel_Init(BatteryModel_HandleTypeDef *battery_model, CellModel_HandleTypeDef* cell_memory_pool, uint16_t cell_count, uint16_t cells_in_series, uint16_t cells_in_parallel)
{
    battery_model->CellCount = cell_count;
    battery_model->CellsInSeries = cells_in_series;
    battery_model->CellsInParallel = cells_in_parallel;
    battery_model->AverageTemperature = 20; // Default temperature for no apparent reason
    battery_model->Cells = cell_memory_pool;
    if (battery_model->Cells == NULL || sizeof(battery_model->Cells) < cell_count * sizeof(CellModel_HandleTypeDef))
    {
        Error_Handler();
    }
    for (int i = 0; i < cell_count; i++)
    {
        battery_model->Cells[i].CellID = i;
        battery_model->Cells[i].TemperatureID = i;
        battery_model->Cells[i].EstimatedVoltage = 0;
        battery_model->Cells[i].MeasuredVoltage = 0;
        battery_model->Cells[i].MeasuredTemperature = 0;
    }
}

void BatteryModel_UpdateMeasured(BatteryModel_HandleTypeDef *battery_model, float *cell_voltages, float *cell_temperatures, float *total_current)
{
    
    for (int i = 0; i < battery_model->CellCount; i++)
    {
        battery_model->Cells[i].MeasuredVoltage = cell_voltages[i];
        battery_model->Cells[i].MeasuredCurrent = total_current[i] / battery_model->CellsInParallel;

        // Only update the measured resistance if the current is above a certain threshold
        // TODO: Should think about doing this only after a certain amount of time
        if(total_current <= 10){
            battery_model->Cells[i].MeasuredRestingVoltage = cell_voltages[i];
        }else{
            battery_model->Cells[i].MeasuredResistance = battery_model->Cells[i].MeasuredRestingVoltage - battery_model->Cells[i].MeasuredVoltage / battery_model->Cells[i].MeasuredCurrent;
        }
        // TODO: Map temperatures to cells
    }


}

void BatteryModel_LoadCellData(BatteryModel_HandleTypeDef *battery_model, float k0, float k1, float k2, float k3, float k4, float discharge_resistance, float charging_resistance, float hysteresis){

    battery_model->K0 = k0;
    battery_model->K1 = k1;
    battery_model->K2 = k2;
    battery_model->K3 = k3;
    battery_model->K4 = k4;
    battery_model->DischargeResistance = discharge_resistance;
    battery_model->ChargingResistance = charging_resistance;
    battery_model->H = hysteresis;

}

void BatteryModel_UpdateEstimates(BatteryModel_HandleTypeDef *battery_model){
    // TODO: Implement the juicy part here
    for(int i = 0; i<battery_model->CellCount; i++){
        CellModel_HandleTypeDef *cell = &battery_model->Cells[i];
        
    }
}

void BatteryModel_InitOCVMaps(BatteryModel_HandleTypeDef *battery_model, float *SOCPoints, uint16_t size, float* voltage_point_memory_pool, uint8_t numOfMaps){
}
