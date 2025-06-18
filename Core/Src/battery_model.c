#include "battery_model.h"
#include "stdlib.h"





void BatteryModel_Configure(BatteryModel_HandleTypeDef *battery_model, uint16_t cell_count, uint16_t cells_in_series, uint16_t nominal_cell_capacity)
{
    if (cell_count > CELL_MEMORY_POOL_SIZE)
    {
        // If this occurs, you have to change the CELL_MEMORY_POOL_SIZE in battery_model.h
        Error_Handler();
    }

    battery_model->CellCount = cell_count;
    battery_model->CellsInSeries = cells_in_series;

    for (int i = 0; i < cell_count; i++)
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

        for (int i = 0; i < battery_model->CellCount; i++)
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

            for (int j = 0; j < battery_model->OCV.Size; j++)
            {
                if (cell_voltages[i] < (battery_model->OCV.VoltagePoints[j] / 1000.0f))
                {
                    battery_model->Cells[i].EstimatedSOC = ((float)j / (float)battery_model->OCV.Size) * 100.0f;
                    battery_model->Cells[i].EstimatedCapacity = (battery_model->Cells[i].EstimatedSOC / 100.0f) * battery_model->Cells[i].NominalCapacity;
                    break;
                }
            }
        }
        battery_model->FirstEstimate = true;
        return;
    }

    for (int i = 0; i < battery_model->CellCount; i++)
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

        battery_model->Cells[i].EstimatedCapacity += battery_model->Cells[i].MeasuredCurrent * dt / 3.6f; // mAh
        battery_model->Cells[i].EstimatedSOC = (battery_model->Cells[i].EstimatedCapacity / battery_model->Cells[i].NominalCapacity) * 100.0f;
    }
}
