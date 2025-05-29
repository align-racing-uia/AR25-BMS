#include "broadcasts.h"

bool SendBroadcastMessage(BMS_BroadcastTypeDef *data)
{

  uint8_t can_data[8] = {0};
  can_data[0] = (data->dc_limit >> 8) & 0xFF;  // Should be DC_limit x 10
  can_data[1] = data->dc_limit & 0xFF;         // Should be DC_limit x 10
  can_data[2] = (data->cc_limit >> 8) & 0xFF;  // Should be CC_limit x 10
  can_data[3] = data->cc_limit & 0xFF;         // Should be CC_limit x 10
  can_data[4] = (data->high_temp >> 8) & 0xFF; // Should be avg_cell_temp x 10
  can_data[5] = data->high_temp & 0xFF;        // Should be avg_cell_temp x 10
  can_data[6] = (data->soc >> 8) & 0xFF;       // Should be soc x 10
  can_data[7] = data->soc & 0xFF;              // Should be soc x 10
  uint32_t can_id = Align_CombineCanId(data->packet_id, data->node_id, data->extended);
  Align_CAN_AddToBuffer(data->hfdcan, can_id, can_data, 8, data->extended);

  can_data[0] = data->active_faults;          // Should be the active faults
  can_data[1] = data->sdc_voltage_raw >> 8;   // Should be the SDC voltage 0xFF00
  can_data[2] = data->sdc_voltage_raw & 0xFF; // Should be the SDC voltage 0x00FF
  can_data[3] = data->avg_cycle_time >> 8;    // Should be the cycle time in ms
  can_data[4] = data->avg_cycle_time & 0xFF;  // Should be the cycle time in ms
  can_id = Align_CombineCanId(data->packet_id + 1, data->node_id, data->extended);

  Align_CAN_AddToBuffer(data->hfdcan, can_id, can_data, 5, data->extended); // Send the message again to make sure it is sent

  return true;
}

bool SendVoltageData(BQ_VoltageBroadcastTypeDef *data)
{
  uint8_t cell_voltage_data[8] = {0};
  uint8_t full_messages = data->bq_handle->NumOfCellsEach * data->bq_handle->NumOfSlaves / 4;
  uint8_t partial_message = data->bq_handle->NumOfCellsEach * data->bq_handle->NumOfSlaves % 4; // Check if there is a partial message
  uint8_t i = 0;
  while (full_messages > 0 || partial_message > 0)
  {
    if (full_messages > 0)
    {

      *((uint16_t *)cell_voltage_data) = (uint16_t)(data->bq_handle->CellVoltages[i * 4] * 10000.0);           // Convert to mV
      *((uint16_t *)(cell_voltage_data + 2)) = (uint16_t)(data->bq_handle->CellVoltages[i * 4 + 1] * 10000.0); // Convert to mV
      *((uint16_t *)(cell_voltage_data + 4)) = (uint16_t)(data->bq_handle->CellVoltages[i * 4 + 2] * 10000.0); // Convert to mV
      *((uint16_t *)(cell_voltage_data + 6)) = (uint16_t)(data->bq_handle->CellVoltages[i * 4 + 3] * 10000.0); // Convert to mV
      full_messages--;
      Align_CAN_AddToBuffer(data->hfdcan, Align_CombineCanId(data->packet_id + i, data->node_id, data->extended), cell_voltage_data, 8, data->extended); // Send the message to the CAN bus
    }
    else
    {
      for (uint8_t j = 0; j < partial_message; j++)
      {
        *((uint16_t *)(cell_voltage_data + j * 2)) = (uint16_t)(data->bq_handle->CellVoltages[i * 4 + j] * 10000.0); // Convert to mV
      }
      Align_CAN_AddToBuffer(data->hfdcan, Align_CombineCanId(data->packet_id + i, data->node_id, data->extended), cell_voltage_data, 2 * partial_message, data->extended); // Send the message to the CAN bus
      partial_message = 0;
    }

    i++;
  }
  return true;
}