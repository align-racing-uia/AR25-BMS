#include "battery_model.h"
#include "stdlib.h"
#include "main.h"




void BatteryModel_Configure(BatteryModel_HandleTypeDef *battery_model, uint16_t cells_in_series, uint16_t nominal_cell_capacity)
{
    if (cells_in_series > CELL_MEMORY_POOL_SIZE)
    {
        // If this occurs, you have to change the CELL_MEMORY_POOL_SIZE in battery_model.h
        Error_Handler();
    }

    battery_model->CellsInSeries = cells_in_series;

    for (int i = 0; i < cells_in_series; i++)
    {
        battery_model->Cells[i].MeasuredVoltage = 0;
        battery_model->Cells[i].MeasuredTemperature = 0;
        battery_model->Cells[i].NominalCapacity = nominal_cell_capacity; // Set the nominal capacity of the cell
    }
}

void BatteryModel_BindMemory(BatteryModel_HandleTypeDef *battery_model, CellModel_HandleTypeDef *cell_memory_pool, float* ocv_map_voltage_points, size_t ocv_map_size)
{
    if (cell_memory_pool == NULL && ocv_map_voltage_points == NULL && ocv_map_size < 1)
    {
        Error_Handler();
    }

    battery_model->Cells = cell_memory_pool;
    battery_model->OCV.VoltagePoints = ocv_map_voltage_points;
    battery_model->OCV.Size = ocv_map_size;

}

void BatteryModel_Update(BatteryModel_HandleTypeDef *battery_model, float *cell_voltages, float *cell_temperatures, float total_current, uint16_t cycle_time)
{
    float dt = ((float)(cycle_time)) / 1000.0f;

    // If SOC hasnt been estimated yet, use OCV maps to estimate initial SOC
    if (!battery_model->FirstEstimate)
    {

        for (int i = 0; i < battery_model->CellsInSeries; i++)
        {
            if (cell_voltages[i] == 0)
            {
                // If the cell voltage wait until first measurement is loaded
                return;
            }
            // TODO: Ensure that the cell voltage measurements align with the temperature measurements
            battery_model->Cells[i].MeasuredVoltage = cell_voltages[i];
            battery_model->Cells[i].MeasuredTemperature = cell_temperatures[i];
            battery_model->Cells[i].MeasuredCurrent = total_current; // Treat each set of cells in parallel as one cell for now
            float lowest_soc = 100.0f; // Start with the highest SOC possible 
            for (int j = 0; j < battery_model->OCV.Size; j++)
            {
                if (cell_voltages[i] < (battery_model->OCV.VoltagePoints[j] / 1000.0f))
                {
                    battery_model->Cells[i].EstimatedSOC = ((float)j / (float)battery_model->OCV.Size) * 100.0f;
                    battery_model->Cells[i].EstimatedCapacity = (battery_model->Cells[i].EstimatedSOC / 100.0f) * battery_model->Cells[i].NominalCapacity;

                    break;
                }else{
                    battery_model->Cells[i].EstimatedSOC = 100.0f;
                    battery_model->Cells[i].EstimatedCapacity = battery_model->Cells[i].NominalCapacity; // If the voltage is above the highest voltage in the OCV map, set SOC to 100%
                }
                battery_model->EstimatedSOC = lowest_soc; // Set the estimated SOC to the lowest SOC of the cells, as we dont want anything to burn
            }
        }
        battery_model->FirstEstimate = true;
        return;
    }
    float lowest_soc = 100.0f; // Start with the highest SOC possible 

    for (int i = 0; i < battery_model->CellsInSeries; i++)
    {
        battery_model->Cells[i].MeasuredVoltage = cell_voltages[i];
        battery_model->Cells[i].MeasuredCurrent = total_current; // Divide the total current by the number of cells in parallel to get the current per cell
        battery_model->Cells[i].MeasuredTemperature = cell_temperatures[i];

        // Only update the measured resistance if the current is above a certain threshold
        // TODO: Should think about doing this only after a certain amount of time
        if (total_current <= 5.0f)
        {
            battery_model->Cells[i].MeasuredRestingVoltage = cell_voltages[i];
        }
        else
        {
            //Apply Ohm's law to make a simple estimate
            battery_model->Cells[i].EstimatedResistance = (battery_model->Cells[i].MeasuredRestingVoltage - battery_model->Cells[i].MeasuredVoltage) / battery_model->Cells[i].MeasuredCurrent;
        }
        // TODO: Map temperatures to cells
        battery_model->Cells[i].EstimatedCapacity += battery_model->Cells[i].MeasuredCurrent * dt / 36.0f; // A*10 -> mAh
        battery_model->Cells[i].EstimatedSOC = (battery_model->Cells[i].EstimatedCapacity / battery_model->Cells[i].NominalCapacity) * 100.0f;
        if (battery_model->Cells[i].EstimatedSOC < lowest_soc)
        {
            lowest_soc = battery_model->Cells[i].EstimatedSOC; // Find the lowest SOC for the cell
        }
    }
    battery_model->EstimatedSOC = lowest_soc; // Set the estimated SOC to the lowest SOC of the cells, as we dont want anything to burn
}
