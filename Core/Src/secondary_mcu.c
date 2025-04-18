#include "secondary_mcu.h"

uint8_t secondary_mcu_recieve_index = 1; // Flag to check if the secondary MCU is ready
// Start at 1 to wrap to 0 the first run

void SecondaryMCU_RecieveCallback(DMA_HandleTypeDef *hdma) {
    secondary_mcu_recieve_index = !secondary_mcu_recieve_index; // Alternate the index
}
