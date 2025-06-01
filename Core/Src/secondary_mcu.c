#include "secondary_mcu.h"

extern SecondaryMCU_HandleTypeDef *global_secondary_mcu_handle; // Global handle for the secondary MCU, used in the DMA callback

// Private function defines

static void SecondaryMCU_Recieve(SecondaryMCU_HandleTypeDef *hsecondary);
static void SecondaryMCU_RecieveCallback(DMA_HandleTypeDef *hdma); // Callback function for the DMA transfer complete event

// Private variable definitions
SecondaryMCU_HandleTypeDef *global_secondary_mcu_handle; // Global handle for the secondary MCU, used in the DMA callback

// Global function implementations

void SecondaryMCU_Init(SecondaryMCU_HandleTypeDef *hsecondary, SPI_HandleTypeDef *hspi, GPIO_TypeDef *CsPinPort, uint16_t CsPin)
{

    global_secondary_mcu_handle = hsecondary;         // Set the global handle for the secondary MCU
    hsecondary->hspi = hspi;                          // Set the SPI handle
    hsecondary->CsPinPort = CsPinPort;                // Set the GPIO port for the chip select pin
    hsecondary->CsPin = CsPin;                        // Set the chip select pin number
    hsecondary->RecieveWriteIndex = 0;                // Initialize the write index for the receive buffer
    hsecondary->RecieveReadIndex = 0;                 // Initialize the read index for the receive buffer
    hsecondary->LastRecieveTimestamp = HAL_GetTick(); // Initialize the last receive timestamp to 0

    // Activate callback
    HAL_DMA_RegisterCallback(hsecondary->hspi->hdmarx, HAL_DMA_XFER_CPLT_CB_ID, SecondaryMCU_RecieveCallback); // Register the receive callback for the DMA
}

void SecondaryMCU_RequestState(SecondaryMCU_HandleTypeDef *hsecondary, BMS_TS_StateTypeDef state)
{
    hsecondary->TransmitData.TsRequest = state;    // Get the current state of the tractive system
    HAL_SPI_Transmit_DMA(hsecondary->hspi, (uint8_t *)&hsecondary->TransmitData, sizeof(SecondaryMCU_TransmitTypeDef)); // Transmit the data to the secondary MCU
}

void SecondaryMCU_Update(SecondaryMCU_HandleTypeDef *hsecondary)
{

    // Store the current index in case an update occurs during the update
    uint8_t read_index = hsecondary->RecieveReadIndex; // Get the read index for the receive buffer

    hsecondary->IsConnected = hsecondary->LastRecieveTimestamp + 1000 > HAL_GetTick(); // Check if the secondary MCU is connected, based on the last receive timestamp
    hsecondary->IsSlow = hsecondary->RecieveData[read_index].PingPongTiming > 100;     // Reset the ping-pong timing
    hsecondary->RelayStates = hsecondary->RecieveData[read_index].RelayStates;         // Reset the relay states
    hsecondary->SdcClosed = hsecondary->RecieveData[read_index].SdcClosed;             // Reset the SdcClosed state
}

void SecondaryMCU_Poll(SecondaryMCU_HandleTypeDef *hsecondary)
{

    if (HAL_SPI_GetState(hsecondary->hspi) == HAL_SPI_STATE_READY)
    {
        uint8_t poll = 0xFF; // Polling byte to send to the secondary MCU
        HAL_SPI_TransmitReceive_DMA(hsecondary->hspi, &poll, (uint8_t *)&hsecondary->RecieveData[hsecondary->RecieveWriteIndex], sizeof(SecondaryMCU_TransmitTypeDef)); // Transmit and receive data using DMA                                                                                                                                      
    }
}

// Private function implementations

static void SecondaryMCU_RecieveCallback(DMA_HandleTypeDef *hdma)
{
    if (hdma->ChannelIndex == 6)
    {                                                      // Check if the DMA is for the SPI1 RX
        SecondaryMCU_Recieve(global_secondary_mcu_handle); // Call the receive function for the secondary MCU
    }
}

static void SecondaryMCU_Recieve(SecondaryMCU_HandleTypeDef *hsecondary)
{
    // This function is called when the DMA transfer is complete
    // It will read the data from the SPI peripheral and store it in the receive buffer
    // The receive buffer is a cyclic buffer, so we will use the write index to write the data
    // and the read index to read the data
    hsecondary->RecieveReadIndex = hsecondary->RecieveWriteIndex; // Set the read index to the write index
    hsecondary->RecieveWriteIndex++;

    if (hsecondary->RecieveWriteIndex >= 2)
    {                                      // If the write index is greater than or equal to the size of the receive buffer
        hsecondary->RecieveWriteIndex = 0; // Reset the write index to 0
    }

    hsecondary->LastRecieveTimestamp = HAL_GetTick(); // Update the last receive timestamp
}