#include "battery_model.h"
#include "stdlib.h"

void BatteryModel_Init(BatteryModel_HandleTypeDef *battery_model, CellModel_HandleTypeDef *cell_memory_pool, uint16_t cell_count, uint16_t cells_in_series, uint16_t cells_in_parallel, uint16_t nominal_cell_capacity)
{
    if (cell_count > CELL_MEMORY_POOL_SIZE)
    {
        // If this occurs, you have to change the CELL_MEMORY_POOL_SIZE in battery_model.h
        Error_Handler();
    }
    
    memset(battery_model, 0, sizeof(BatteryModel_HandleTypeDef)); // Clear the memory for the battery model

    battery_model->CellCount = cell_count;
    battery_model->CellsInSeries = cells_in_series;
    battery_model->CellsInParallel = cells_in_parallel;
    battery_model->AverageTemperature = 25; // Default temperature for no apparent reason
    battery_model->Cells = cell_memory_pool;
    if (battery_model->Cells == NULL)
    {
        Error_Handler();
    }
    for (int i = 0; i < cell_count; i++)
    {
        battery_model->Cells[i].MeasuredVoltage = 0;
        battery_model->Cells[i].MeasuredTemperature = 0;
        battery_model->Cells[i].NominalCapacity = nominal_cell_capacity; // Set the nominal capacity of the cell
    }
}

void BatteryModel_Update(BatteryModel_HandleTypeDef *battery_model, float *cell_voltages, float *cell_temperatures, float *total_current, uint16_t current_timestamp)
{
    float dt = ((float)(current_timestamp)) / 1000.0f;

    // If SOC hasnt been estimated yet, use OCV maps to estimate initial SOC
    if(!battery_model->soc_estimated){

        for(int i = 0; i < battery_model->CellCount; i++)
        {
            if(cell_voltages[i] == 0)
            {
                // If the cell voltage wait until first measurement is loaded
                return;

            }
            battery_model->Cells[i].MeasuredVoltage = cell_voltages[i];
            battery_model->Cells[i].MeasuredCurrent = total_current[i] / battery_model->CellsInParallel;
            battery_model->Cells[i].MeasuredTemperature = cell_temperatures[i];

            TempMap_HandleTypeDef *temp_map = NULL;
            for(int y=0; y < battery_model->OCV.NumOfMaps; y++)
            {
                if(battery_model->Cells[i].MeasuredTemperature < battery_model->OCV.TemperatureMaps[y].Temperature)
                {
                    temp_map = &battery_model->OCV.TemperatureMaps[y];
                    break;
                }
            }
            if(temp_map == NULL)
            {
                temp_map = &battery_model->OCV.TemperatureMaps[0];
            }

            for(int j = 0; j < temp_map->Size; j++)
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
        battery_model->soc_estimated = true;
        return;
    }

    for (int i = 0; i < battery_model->CellCount; i++)
    {
        battery_model->Cells[i].MeasuredVoltage = cell_voltages[i];
        battery_model->Cells[i].MeasuredCurrent = *total_current / battery_model->CellsInParallel;
        battery_model->Cells[i].MeasuredTemperature = cell_temperatures[i];
        
        // Only update the measured resistance if the current is above a certain threshold
        // TODO: Should think about doing this only after a certain amount of time
        if (*total_current <= 5.0f)
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

void BatteryModel_LoadECMData(BatteryModel_HandleTypeDef *battery_model, float k0, float k1, float k2, float k3, float k4, float discharge_resistance, float charging_resistance, float hysteresis)
{

    battery_model->K0 = k0;
    battery_model->K1 = k1;
    battery_model->K2 = k2;
    battery_model->K3 = k3;
    battery_model->K4 = k4;
    battery_model->DischargeResistance = discharge_resistance;
    battery_model->ChargingResistance = charging_resistance;
    battery_model->H = hysteresis;
}

void BatteryModel_InitOCVMaps(BatteryModel_HandleTypeDef *battery_model, float *voltage_point_memory_pool, TempMap_HandleTypeDef *temp_map_memory_pool,  uint8_t* temps, uint16_t amount_of_voltage_points, uint8_t num_of_maps)
{

    // Check if the amount of points set are too many
    if (amount_of_voltage_points > TEMP_MAP_POOL_MAX_POINTS || amount_of_voltage_points < 1)
    {
        Error_Handler(); // Hard stop if this fails
    }
    // Check if there are too many maps configured
    if (num_of_maps > TEMP_MAP_POOL_AMOUNT || num_of_maps < 1)
    {
        Error_Handler(); // Hard stop if this fails
    }

    battery_model->OCV.TemperatureMaps = temp_map_memory_pool;

    // Check if the actual memory pool is large enough
    if (battery_model->OCV.TemperatureMaps == NULL)
    {
        Error_Handler();
    }
    // Map the memory for the temperature maps
    for (int i = 0; i < num_of_maps; i++)
    {
        battery_model->OCV.TemperatureMaps[i].VoltagePoints = voltage_point_memory_pool + (i * amount_of_voltage_points);
        battery_model->OCV.TemperatureMaps[i].Size = amount_of_voltage_points;
        battery_model->OCV.TemperatureMaps[i].Temperature = temps[i]; // Set the temperature for the map
    }
    battery_model->OCV.NumOfMaps = amount_of_voltage_points;
}
