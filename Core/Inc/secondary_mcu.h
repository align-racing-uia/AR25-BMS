#ifndef SECONDARY_MCU_H
#define SECONDARY_MCU_H

#include "stdint.h"
#include "stdbool.h"
#include "spi.h"

typedef struct {
    SPI_HandleTypeDef *hspi; // Handle for the SPI peripheral
    GPIO_TypeDef *CsPinPort;  // GPIO port for the chip select pin
    uint16_t CsPin;           // Chip select pin number
    
    // Cyclic data buffers
    SecondaryMCU_TransmitTypeDef TransmitData[3]; // Data to transmit to the secondary MCU
    SecondaryMCU_ResponseTypeDef RecieveData[3];  // Data received from the secondary MCU
    uint8_t TransmitIndex; // Index for the transmit data buffer
    uint8_t RecieveIndex;  // Index for the receive data buffer

    

} SecondaryMCU_HandleTypeDef;

typedef struct
{
    int16_t PingPongDeviation; // The deviation from the last ping-pong signal (optimal time 100ms)
    uint8_t RelayStates;
    bool SdcClosed; // SdcClosed state

} SecondaryMCU_ResponseTypeDef;

typedef struct
{
    uint8_t TsRequest; // Request for the tractive system state
} SecondaryMCU_TransmitTypeDef;


void SecondaryMCU_RecieveCallback(DMA_HandleTypeDef *hdma);

#endif