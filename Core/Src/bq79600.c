#include "bq79600.h"
#include "align-utils.h"

void BQ_Init(BQ_HandleTypeDef* hbq){
    HAL_GPIO_WritePin(hbq->csGPIOx, hbq->csPin, GPIO_PIN_SET);
}

void BQ_Wake(BQ_HandleTypeDef* hbq){
    HAL_GPIO_WritePin(hbq->csGPIOx, hbq->csPin, GPIO_PIN_RESET);
    Align_DelayUs(10);
    HAL_GPIO_WritePin(hbq->csGPIOx, hbq->csPin, GPIO_PIN_SET);
}