#ifndef BQ79600_H
#define BQ79600_H
#include "main.h"

#define BQ_DEVICE_READ 0x80
#define BQ_DEVICE_WRITE 0x90
#define BQ_STACK_READ 0xA0
#define BQ_STACK_WRITE 0xB0
#define BQ_BROAD_READ 0xC0
#define BQ_BROAD_WRITE 0xD0
#define BQ_BROAD_REVERSE_WRITE 0xE0

typedef struct {

    SPI_HandleTypeDef* hspi;
    GPIO_TypeDef* csGPIOx;
    uint16_t csPin;


} BQ_HandleTypeDef;

void BQ_Init(BQ_HandleTypeDef* hbq);

void BQ_Wake(BQ_HandleTypeDef* hbq);


#endif