#include "battery_model.h"
#include "stdlib.h"

float sim_ocv_map[] = {
    2907.03,
    3021.21,
    3116.98,
    3185.15,
    3239.26,
    3284.97,
    3323.05,
    3355.40,
    3382.05,
    3402.41,
    3418.22,
    3431.19,
    3441.68,
    3450.80,
    3459.78,
    3468.63,
    3477.72,
    3487.24,
    3497.56,
    3508.97,
    3519.96,
    3530.81,
    3541.47,
    3551.71,
    3561.86,
    3571.15,
    3579.96,
    3588.14,
    3595.65,
    3603.04,
    3610.06,
    3616.73,
    3623.12,
    3629.34,
    3635.58,
    3641.95,
    3648.20,
    3654.77,
    3661.46,
    3668.54,
    3675.52,
    3683.01,
    3690.63,
    3698.58,
    3706.65,
    3715.03,
    3723.80,
    3732.35,
    3741.03,
    3750.03,
    3758.76,
    3767.81,
    3776.57,
    3785.47,
    3794.56,
    3803.86,
    3813.02,
    3822.78,
    3832.66,
    3842.37,
    3851.54,
    3860.47,
    3868.98,
    3877.00,
    3884.63,
    3892.32,
    3899.32,
    3906.63,
    3913.96,
    3921.29,
    3929.20,
    3937.26,
    3946.02,
    3955.38,
    3965.09,
    3975.40,
    3986.15,
    3997.05,
    4007.91,
    4018.71,
    4029.22,
    4038.90,
    4048.19,
    4056.58,
    4063.70,
    4069.65,
    4074.40,
    4078.24,
    4081.59,
    4084.89,
    4088.05,
    4091.90,
    4095.77,
    4100.49,
    4106.10,
    4112.92,
    4121.36,
    4131.89,
    4145.73,
    4163.89,
    4183.32};

void BatteryModel_Configure(BatteryModel_HandleTypeDef *battery_model, uint16_t cell_count, uint16_t cells_in_series, uint16_t cells_in_parallel, uint16_t nominal_cell_capacity)
{
    if (cell_count > CELL_MEMORY_POOL_SIZE)
    {
        // If this occurs, you have to change the CELL_MEMORY_POOL_SIZE in battery_model.h
        Error_Handler();
    }

    battery_model->CellCount = cell_count;
    battery_model->CellsInSeries = cells_in_series;
    battery_model->CellsInParallel = cells_in_parallel;

    for (int i = 0; i < cell_count; i++)
    {
        battery_model->Cells[i].MeasuredVoltage = 0;
        battery_model->Cells[i].MeasuredTemperature = 0;
        battery_model->Cells[i].NominalCapacity = nominal_cell_capacity; // Set the nominal capacity of the cell
    }
}

void BatteryModel_BindMemory(BatteryModel_HandleTypeDef *battery_model, CellModel_HandleTypeDef *cell_memory_pool)
{
    battery_model->Cells = cell_memory_pool;
    if (battery_model->Cells == NULL)
    {
        Error_Handler();
    }

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
            battery_model->Cells[i].MeasuredVoltage = cell_voltages[i];
            battery_model->Cells[i].MeasuredCurrent = total_current / battery_model->CellsInParallel; // Divide the total current by the number of cells in parallel to get the current per cell
            battery_model->Cells[i].MeasuredTemperature = cell_temperatures[i];

            TempMap_HandleTypeDef *temp_map = NULL;
            for (int y = 0; y < battery_model->OCV.NumOfMaps; y++)
            {
                if (battery_model->Cells[i].MeasuredTemperature < battery_model->OCV.TemperatureMaps[y].Temperature)
                {
                    temp_map = &battery_model->OCV.TemperatureMaps[y];
                    break;
                }
            }
            if (temp_map == NULL)
            {
                temp_map = &battery_model->OCV.TemperatureMaps[0];
            }

            for (int j = 0; j < temp_map->Size; j++)
            {
                if (cell_voltages[i] < (temp_map->VoltagePoints[j] / 1000.0f))
                {
                    // Very crude linear interpolation for now
                    battery_model->Cells[i].EstimatedSOC = ((float)j / (float)temp_map->Size) * 100.0f;
                    battery_model->Cells[i].EstimatedCapacity = (battery_model->Cells[i].EstimatedSOC / 100.0f) * battery_model->Cells[i].NominalCapacity;
                    break;
                }
            }
        }
        battery_model->LastCycle = HAL_GetTick();
        battery_model->FirstEstimate = true;
        return;
    }

    for (int i = 0; i < battery_model->CellCount; i++)
    {
        battery_model->Cells[i].MeasuredVoltage = cell_voltages[i];
        battery_model->Cells[i].MeasuredCurrent = total_current / battery_model->CellsInParallel; // Divide the total current by the number of cells in parallel to get the current per cell
        battery_model->Cells[i].MeasuredTemperature = cell_temperatures[i];

        // Only update the measured resistance if the current is above a certain threshold
        // TODO: Should think about doing this only after a certain amount of time
        if (total_current <= 5.0f)
        {
            battery_model->Cells[i].MeasuredRestingVoltage = cell_voltages[i];
        }
        else
        {
            battery_model->Cells[i].EstimatedResistance = battery_model->Cells[i].MeasuredRestingVoltage - battery_model->Cells[i].MeasuredVoltage / battery_model->Cells[i].MeasuredCurrent;
        }
        // TODO: Map temperatures to cells

        battery_model->Cells[i].EstimatedCapacity += battery_model->Cells[i].MeasuredCurrent * dt / 3.6f; // mAh
        battery_model->Cells[i].EstimatedSOC = (battery_model->Cells[i].EstimatedCapacity / battery_model->Cells[i].NominalCapacity) * 100.0f;
    }
}


void BatteryModel_LoadOCVMap(BatteryModel_HandleTypeDef *battery_model, float* voltage_points, uint8_t temp, uint8_t map_index, uint16_t num_of_voltage_points)
{

}
