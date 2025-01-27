#ifndef BQ79600_H
#define BQ79600_H
#include "main.h"
#include "stdbool.h"

#define BQ_TIMEOUT 2000

// BQ79600 Own ID
#define BQ_SELF_ID 0x00

// Initialization Bytes
#define BQ_DEVICE_READ 0x80
#define BQ_DEVICE_WRITE 0x90
#define BQ_STACK_READ 0xA0
#define BQ_STACK_WRITE 0xB0
#define BQ_BROAD_READ 0xC0
#define BQ_BROAD_WRITE 0xD0
#define BQ_BROAD_REVERSE_WRITE 0xE0

// Some important registers
#define BQ_CONTROL1 0x0309

#define BQ_CONTROL1_SEND_WAKE 1<<5

typedef struct {

    SPI_HandleTypeDef* hspi;
    GPIO_TypeDef* csGPIOx;
    GPIO_TypeDef* spiRdyGPIOx;
    GPIO_TypeDef* mosiGPIOx;
    uint16_t csPin;
    uint16_t spiRdyPin;
    uint16_t mosiPin;


} BQ_HandleTypeDef;

void BQ_Init(BQ_HandleTypeDef* hbq);

void BQ_Wake(BQ_HandleTypeDef* hbq);

bool BQ_SpiRdy(BQ_HandleTypeDef* hbq);

uint8_t BQ_Read(BQ_HandleTypeDef* hbq, uint8_t *dataOut, uint8_t deviceId, uint16_t regAddr, uint8_t dataLength, uint8_t readType);

uint8_t BQ_Write(BQ_HandleTypeDef* hbq, uint8_t *inData, uint8_t deviceId, uint16_t regAddr, uint8_t dataLength, uint8_t writeType);


#endif