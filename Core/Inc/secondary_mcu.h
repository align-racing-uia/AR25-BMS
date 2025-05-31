#ifndef SECONDARY_MCU_H
#define SECONDARY_MCU_H

#include "stdint.h"
#include "stdbool.h"
#include "spi.h"

typedef struct
{
    int16_t PingPongTiming; // The time between the ping-pong exchanges, in milliseconds
    uint8_t RelayStates;
    bool SdcClosed; // SdcClosed state

} SecondaryMCU_ResponseTypeDef;

typedef struct
{
    uint8_t TsRequest; // Request for the tractive system state
} SecondaryMCU_TransmitTypeDef;

// There can only be one instance of the secondary MCU, because of the global flag needed for the DMA callback
typedef struct {
    SPI_HandleTypeDef *hspi; // Handle for the SPI peripheral
    GPIO_TypeDef *CsPinPort;  // GPIO port for the chip select pin
    uint16_t CsPin;           // Chip select pin number

    bool IsConnected; // Flag to indicate if the secondary MCU is connected
    bool IsSlow; // Flag to indicate if the secondary MCU ping-pong is slow
    bool SdcClosed; // Flag to indicate if the SdcClosed is connected
    uint8_t RelayStates; // States of the relays
    
    SecondaryMCU_TransmitTypeDef TransmitData; // Data to transmit to the secondary MCU

    // Cyclic (Alternating) data buffer
    SecondaryMCU_ResponseTypeDef RecieveData[2];  // Data received from the secondary MCU
    uint8_t RecieveWriteIndex;  // Index for the receive data buffer
    uint8_t RecieveReadIndex;   // Index for reading the receive data buffer

    uint32_t LastRecieveTimestamp; // Timestamp of the last receive operation

} SecondaryMCU_HandleTypeDef;


void SecondaryMCU_Init(SecondaryMCU_HandleTypeDef *hsecondary, SPI_HandleTypeDef *hspi, GPIO_TypeDef *CsPinPort, uint16_t CsPin);
void SecondaryMCU_RequestState(SecondaryMCU_HandleTypeDef *hsecondary, TS_StateTypeDef state);
void SecondaryMCU_Update(SecondaryMCU_HandleTypeDef *hsecondary);
void SecondaryMCU_Poll(SecondaryMCU_HandleTypeDef *hsecondary);


#endif