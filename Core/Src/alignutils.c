#include "alignutils.h"
#include "main.h"
#include "tim.h"

void Align_DelayUs(uint32_t us){

  uint32_t begin = htim2.Instance->CNT;
  uint32_t target = begin + us;
  while(htim2.Instance->CNT < target){}

}