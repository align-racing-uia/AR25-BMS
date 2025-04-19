#ifndef SECONDARY_MCU_H
#define SECONDARY_MCU_H

#include "stdint.h"

typedef struct
{
    uint16_t SDCVoltageRaw; // Voltage of the SDC
    int16_t PingPongDeviation; // The deviation from the last ping-pong signal (optimal time 250ms)
    uint8_t RelayStates;

} SecondaryMCU_ResponseTypeDef;

typedef struct
{
    uint8_t RelayRequest;
} SecondaryMCU_TransmitTypeDef;

#endif