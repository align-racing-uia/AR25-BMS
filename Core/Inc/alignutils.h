#ifndef ALIGN_UTILS_H
#define ALIGN_UTILS_H

#include "stdint.h"
#include "align.h"

// Simple delay function that takes microseconds 
// Configured around TIM2 with a 1MHz signal, with a overflow reset every 1000 million cycles
// Max delay is therefore 1000 seconds

void Align_DelayUs(uint32_t us);

#endif