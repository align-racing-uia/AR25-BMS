#include "battery_model.h"
#include "stdlib.h"

void BatteryModel_Init(BatteryModel_HandleTypeDef *battery_model, uint16_t cell_count)
{
    battery_model->CellCount = cell_count;
    battery_model->Cells = (CellModel_HandleTypeDef *)malloc(cell_count * sizeof(CellModel_HandleTypeDef));
    if (battery_model->Cells == NULL)
    {
        Error_Handler();
    }
    for (int i = 0; i < cell_count; i++)
    {
        battery_model->Cells[i].CellID = i;
        battery_model->Cells[i].TemperatureID = i;
        battery_model->Cells[i].EstimatedVoltage = 0;
        battery_model->Cells[i].EstimatedCurrent = 0;
        battery_model->Cells[i].MeasuredVoltage = 0;
        battery_model->Cells[i].MeasuredTemperature = 0;
    }
}

void BatteryModel_UpdateMeasured(BatteryModel_HandleTypeDef *battery_model, float *cell_voltages, float *die_temperatures)
{
    
    
    for (int i = 0; i < battery_model->CellCount; i++)
    {
        battery_model->Cells[i].MeasuredVoltage = cell_voltages[i];
        // TODO: Map temperatures to cells 
    }


}