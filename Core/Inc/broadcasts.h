#ifndef _BROADCASTS_H
#define _BROADCASTS_H

#include "stdbool.h"
#include "bq79600.h"
#include "fdcan.h"

typedef struct
{
  FDCAN_HandleTypeDef *hfdcan; // The FDCAN handle to use for the broadcast
  uint32_t node_id;
  uint32_t packet_id;
  uint16_t cc_limit;
  uint16_t dc_limit;        // DC limit in A * 10
  uint16_t high_temp;       // Highest temperature in C * 10
  uint16_t soc;             // State of charge in % * 10
  uint16_t sdc_voltage_raw; // SdcClosed voltage raw value
  uint16_t avg_cycle_time;  // Cycle time in ms
  uint8_t active_faults;
  bool extended; // Extended ID or not

} BMS_BroadcastTypeDef;

typedef struct
{
  FDCAN_HandleTypeDef *hfdcan; // The FDCAN handle to use for the broadcast
  uint32_t node_id;
  uint32_t packet_id;
  BQ_HandleTypeDef *bq_handle; // The BQ handle to use for the broadcast
  bool extended;               // Extended ID or not

} BQ_VoltageBroadcastTypeDef;

bool SendBroadcastMessage(BMS_BroadcastTypeDef *data);
bool SendVoltageData(BQ_VoltageBroadcastTypeDef *data);

#endif // _BROADCASTS_H