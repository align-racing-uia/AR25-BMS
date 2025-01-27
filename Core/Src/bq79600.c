#include "bq79600.h"

void BQ_Init(BQ_HandleTypeDef* hbq){
    HAL_GPIO_WritePin(hbq->csGPIOx, hbq->csPin, GPIO_PIN_SET);
}

void BQ_Wake(BQ_HandleTypeDef* hbq){
    HAL_GPIO_WritePin(hbq->csGPIOx, hbq->csPin, GPIO_PIN_RESET);
    HAL_Delay(1); // Minimum of 
    HAL_GPIO_WritePin(hbq->csGPIOx, hbq->csPin, GPIO_PIN_SET);
}