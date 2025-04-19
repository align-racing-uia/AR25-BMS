#include "secondary_mcu.h"

uint8_t secondary_mcu_recieve_index = 1; // Flag to check if the secondary MCU is ready
// Start at 1 to wrap to 0 the first run

pSPI_CallbackTypeDef SecondaryMCU_RecieveCallback(SPI_HandleTypeDef *hspi){
    if (hspi->Instance == SPI1)
    {
        secondary_mcu_recieve_index = !secondary_mcu_recieve_index; // Alternate the index
    }
    return NULL;
}
