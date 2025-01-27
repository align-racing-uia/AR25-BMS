#include "bq79600.h"
#include "align-utils.h"
#include "spi.h"

void BQ_Init(BQ_HandleTypeDef* hbq){
    HAL_GPIO_WritePin(hbq->csGPIOx, hbq->csPin, GPIO_PIN_SET);
}

void BQ_Wake(BQ_HandleTypeDef* hbq){
    HAL_GPIO_WritePin(hbq->csGPIOx, hbq->csPin, GPIO_PIN_RESET);
    Align_DelayUs(2); // Atleast 2 us
    HAL_GPIO_WritePin(hbq->mosiGPIOx, hbq->mosiPin, GPIO_PIN_RESET);
    HAL_Delay(3); // Atleast 2.75 ms
    HAL_GPIO_WritePin(hbq->mosiGPIOx, hbq->mosiPin, GPIO_PIN_SET);
    Align_DelayUs(2); // Atleast 2 us
    HAL_GPIO_writePin(hbq->csGPIOx, hbq->csPin, GPIO_PIN_SET);
    HAL_Delay(4); // Atleast 3.75 us
    uint8_t data[1] = {BQ_CONTROL1_SEND_WAKE}; // Data to send to CONTROL1[SEND_WAKE]
    BQ_Write(hbq, &data, BQ_SELF_ID, BQ_CONTROL1, 1, BQ_DEVICE_WRITE);

    // Wait 1.6ms + 10ms pr device on bus
    // 10x 1.6+10 = 116ms
    
    HAL_Delay(120);
}

bool BQ_SpiRdy(BQ_HandleTypeDef* hbq){
    return HAL_GPIO_ReadPin(hbq->spiRdyGPIOx, hbq->spiRdyPin) == GPIO_PIN_SET;
}

// Output:
// 0 - Everything is ok
// 1 - SPI not ready
uint8_t BQ_Read(BQ_HandleTypeDef* hbq, uint8_t *dataOut, uint8_t deviceId, uint16_t regAddr, uint8_t dataLength, uint8_t readType){
    if(BQ_SpiRdy(hbq) != true){
        return 1;
    }
    HAL_GPIO_WritePin(hbq->csGPIOx, hbq->csPin, GPIO_PIN_RESET);
    Align_DelayUs(1);

    return 0;
}

// Output:
// 0 - Everything is ok
// 1 - SPI not ready
// 2 - Invalid writeType
// 3 - Too much data, over 28 bytes
uint8_t BQ_Write(BQ_HandleTypeDef* hbq, uint8_t *inData, uint8_t deviceId, uint16_t regAddr, uint8_t dataLength, uint8_t writeType){
    if(BQ_SpiRdy(hbq) != true){
        return 1;
    }
    if((writeType != BQ_DEVICE_WRITE) && (writeType != BQ_DEVICE_WRITE) && (writeType != BQ_DEVICE_WRITE)){
        return 2;
    }
    if(dataLength > 28){
        return 3;
    }
    // To limit the program size for now
    uint8_t writeData[32] = {0};

    uint8_t writeSize = 0;
    writeData[writeSize] = writeType;
    writeSize++;

    if(writeType == BQ_DEVICE_WRITE){
        writeData[1] = deviceId;
        writeSize++;
    }
    writeData[writeSize] = (uint8_t) (regAddr << 8);
    writeSize++;
    writeData[writeSize] = (uint8_t) regAddr;
    writeSize++;

    for(int i=0; i<dataLength; i++){
        writeData[writeSize] = inData[i];
        writeSize++;
    }  

    // Whole transmit should be ready here

    HAL_GPIO_WritePin(hbq->csGPIOx, hbq->csPin, GPIO_PIN_RESET);
    Align_DelayUs(1); // Atleast 50ns, we dont have that kind of resolution

    HAL_SPI_Transmit(hbq->hspi, &writeData, writeSize, BQ_TIMEOUT);

    return 0;
}