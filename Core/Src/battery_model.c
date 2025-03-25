#include "battery_model.h"

void BatteryModel_Init(BatteryModel_HandleTypeDef *battery_model, uint8_t cell_count)
{
    battery_model->CellCount = cell_count;
    battery_model->Cells = (CellModel_HandleTypeDef *)malloc(cell_count * sizeof(CellModel_HandleTypeDef));
    if (battery_model->Cells == NULL)
    {
        while(true){
            // If this fails, theres no need to continue
        }
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

void BatteryModel_UpdateMeasured(BatteryModel_HandleTypeDef *battery_model, uint16_t *cell_voltages, int8_t *die_temperatures)
{
    
    
    for (int i = 0; i < battery_model->CellCount; i++)
    {
        battery_model->Cells[i].MeasuredVoltage = cell_voltages[i];
        // TODO: Map temperatures to cells 
    }


}