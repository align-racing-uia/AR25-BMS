#ifndef BQ79600_H
#define BQ79600_H
#include "main.h"
#include "stdbool.h"

// CONFIGS

#define TOTALBOARDS 2 // Including base


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
#define BQ_DIR0_ADDR 0x0306
#define BQ_COMM_CTRL 0x0308
#define BQ_OTP_ECC_DATAIN1 0x0343
#define BQ_OTP_ECC_DATAIN2 0x0344
#define BQ_OTP_ECC_DATAIN3 0x0345
#define BQ_OTP_ECC_DATAIN4 0x0346
#define BQ_OTP_ECC_DATAIN5 0x0347
#define BQ_OTP_ECC_DATAIN6 0x0348
#define BQ_OTP_ECC_DATAIN7 0x0349
#define BQ_OTP_ECC_DATAIN8 0x034A

#define BQ_CONTROL1_SEND_WAKE 1<<5
#define BQ_CONTROL1_AA 1<<0

typedef struct {

    SPI_HandleTypeDef* hspi;
    GPIO_TypeDef* csGPIOx;
    GPIO_TypeDef* spiRdyGPIOx;
    GPIO_TypeDef* mosiGPIOx;
    GPIO_TypeDef* nFaultGPIOx;
    uint16_t csPin;
    uint16_t spiRdyPin;
    uint16_t mosiPin;
    uint16_t nFaultPin;


} BQ_HandleTypeDef;

extern uint8_t bqOutputBuffer[128*TOTALBOARDS];

void BQ_Init(BQ_HandleTypeDef* hbq);

void BQ_WakePing(BQ_HandleTypeDef* hbq);
void BQ_WakeMsg(BQ_HandleTypeDef* hbq);

void BQ_ClearComm(BQ_HandleTypeDef* hbq);
void BQ_AutoAddress(BQ_HandleTypeDef* hbq);

bool BQ_SpiRdy(BQ_HandleTypeDef* hbq);

uint8_t BQ_Read(BQ_HandleTypeDef* hbq, uint8_t *pOut, uint8_t deviceId, uint16_t regAddr, uint8_t dataLength, uint8_t readType);

uint8_t BQ_Write(BQ_HandleTypeDef* hbq, uint8_t *inData, uint8_t deviceId, uint16_t regAddr, uint8_t dataLength, uint8_t writeType);



#endif