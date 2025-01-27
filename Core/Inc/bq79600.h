#ifndef BQ79600_H
#define BQ79600_H
#include "main.h"

typedef struct {

    SPI_HandleTypeDef* hspi;
    GPIO_TypeDef* csGPIOx;
    uint16_t csPin;


} BQ_HandleTypeDef;

void BQ_Init(BQ_HandleTypeDef* hbq);

void BQ_Wake(BQ_HandleTypeDef* hbq);


#endif