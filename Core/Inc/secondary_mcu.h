#ifndef SECONDARY_MCU_H
#define SECONDARY_MCU_H

#include "stdint.h"
#include "stdbool.h"
#include "spi.h"

typedef struct {
    SPI_HandleTypeDef *hspi; // Handle for the SPI peripheral
    GPIO_TypeDef *CsPinPort;  // GPIO port for the chip select pin
    uint16_t CsPin;           // Chip select pin number
} SecondaryMCU_HandleTypeDef;

typedef struct
{
    uint16_t SDCVoltageRaw; // Voltage of the SdcClosed
    int16_t PingPongDeviation; // The deviation from the last ping-pong signal (optimal time 250ms)
    uint8_t RelayStates;

} SecondaryMCU_ResponseTypeDef;

typedef struct
{
    uint8_t RelayRequest;
} SecondaryMCU_TransmitTypeDef;

extern uint8_t secondary_mcu_recieve_index; // Alternate what buffer is read / written to

void SecondaryMCU_RecieveCallback(DMA_HandleTypeDef *hdma);

#endif