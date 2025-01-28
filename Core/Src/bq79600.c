#include "bq79600.h"
#include "align-utils.h"
#include "spi.h"
#include "crc.h"

// Because of a odd design requirement this is needed..
// We sometimes need full control of the MOSI Pin between SPI commands
void BQ_SetMosiGPIO(BQ_HandleTypeDef* hbq){

    HAL_GPIO_DeInit(hbq->mosiGPIOx, hbq->mosiPin);
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = hbq->mosiPin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(hbq->mosiGPIOx, &GPIO_InitStruct);
    

}

// We should always give back control after using it
void BQ_SetMosiSPI(BQ_HandleTypeDef* hbq){
    
    HAL_GPIO_DeInit(hbq->mosiGPIOx, hbq->mosiPin);
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = hbq->mosiPin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(hbq->mosiGPIOx, &GPIO_InitStruct);

}

void BQ_SetMosiIdle(BQ_HandleTypeDef* hbq){
    BQ_SetMosiGPIO(hbq);
    HAL_GPIO_WritePin(hbq->mosiGPIOx,hbq->mosiPin, GPIO_PIN_SET);
}

void BQ_ClearComm(BQ_HandleTypeDef* hbq){
    BQ_SetMosiSPI(hbq);

    HAL_GPIO_WritePin(hbq->csGPIOx, hbq->csPin, GPIO_PIN_RESET);
    Align_DelayUs(100);
    uint8_t data[1] = {0};
    HAL_SPI_Transmit(hbq->hspi, data, 1, BQ_TIMEOUT);
    Align_DelayUs(100);
    HAL_GPIO_WritePin(hbq->csGPIOx, hbq->csPin, GPIO_PIN_SET);

    BQ_SetMosiIdle(hbq); // Should always set Mosi Idle when not sending commands
}

void BQ_Wake(BQ_HandleTypeDef* hbq){

    // We need to be able to control MOSI through GPIO during this
    BQ_SetMosiGPIO(hbq);

    HAL_GPIO_WritePin(hbq->csGPIOx, hbq->csPin, GPIO_PIN_RESET);
    Align_DelayUs(2); // Atleast 2 us

    HAL_GPIO_WritePin(hbq->mosiGPIOx, hbq->mosiPin, GPIO_PIN_RESET);
    HAL_Delay(3); // Atleast 2.75 ms
    HAL_GPIO_WritePin(hbq->mosiGPIOx, hbq->mosiPin, GPIO_PIN_SET);
    Align_DelayUs(2); // Atleast 2 us

    HAL_GPIO_WritePin(hbq->csGPIOx, hbq->csPin, GPIO_PIN_SET);
    HAL_Delay(4); // Atleast 3.75 us

    // We also need Mosi on GPIO this call
    BQ_SetMosiSPI(hbq);

    // Only normal thing that happens during this wake up call..
    uint8_t data[1] = {BQ_CONTROL1_SEND_WAKE}; // Data to send to CONTROL1[SEND_WAKE]
    BQ_Write(hbq, data, BQ_SELF_ID, BQ_CONTROL1, 1, BQ_DEVICE_WRITE);

    // Wait 1.6ms + 10ms pr device on bus
    // 10x 1.6+10 = 116ms
    
    HAL_Delay(120);
}

bool BQ_SpiRdy(BQ_HandleTypeDef* hbq){
    return HAL_GPIO_ReadPin(hbq->spiRdyGPIOx, hbq->spiRdyPin) == GPIO_PIN_SET;
}


void BQ_AutoAddress(BQ_HandleTypeDef* hbq){

    uint8_t data[1] = {0};    
    for(int i=0; i<8; i++){ // Luckly all the registers are after eachother
        BQ_Write(hbq, data, BQ_SELF_ID, BQ_OTP_ECC_DATAIN1+i, 1, BQ_STACK_WRITE);
    }
    data[0] = BQ_CONTROL1_AA;
    BQ_Write(hbq, data, BQ_SELF_ID, BQ_CONTROL1, 1, BQ_BROAD_WRITE);
    for(int i=0; i<4; i++){
        data[0] = i;
        BQ_Write(hbq, data, BQ_SELF_ID, BQ_DIR0_ADDR, 1, BQ_BROAD_WRITE);
    }
    data[0] = 0x02;
    BQ_Write(hbq, data, BQ_SELF_ID, BQ_COMM_CTRL, 1, BQ_BROAD_WRITE);
    data[0] = 0x03;
    BQ_Write(hbq, data, 1, BQ_COMM_CTRL, 1, BQ_DEVICE_WRITE);
    
    for(int i=0; i<8; i++){
        BQ_Read(hbq, data, BQ_SELF_ID, BQ_OTP_ECC_DATAIN1+i, 0, BQ_STACK_READ);
    }
    
    

}


// Output:
// 0 - Everything is ok
// 1 - SPI not ready, and timeout
// 2 - Invalid read type
// 3 - Asking for too much data (max 127 bytes)
// 4 - Recieve error
uint8_t BQ_Read(BQ_HandleTypeDef* hbq, uint8_t *dataOut, uint8_t deviceId, uint16_t regAddr, uint8_t dataLength, uint8_t readType){
    uint32_t start = HAL_GetTick();
    while(BQ_SpiRdy(hbq) != true){
        if(HAL_GetTick() > start + BQ_TIMEOUT){
            return 1;
        }
    }
    if((readType != BQ_DEVICE_READ) && (readType != BQ_STACK_READ) && (readType != BQ_BROAD_READ)){
        return 2;
    }
    if(dataLength > 127){
        return 3;
    }
    HAL_GPIO_WritePin(hbq->csGPIOx, hbq->csPin, GPIO_PIN_RESET);
    Align_DelayUs(1);

    uint8_t writeData[7] = {0};
    uint8_t writeSize = 0;

    writeData[writeSize] = readType;
    writeSize++;
    if(readType == BQ_DEVICE_READ){
        writeData[writeSize] = deviceId;
        writeSize++;
    }
    writeData[writeSize] = (uint8_t) (regAddr >> 8);
    writeSize++;
    writeData[writeSize] = (uint8_t) (regAddr);
    writeSize++;
    writeData[writeSize] = dataLength;
    writeSize++;
    uint16_t crc = HAL_CRC_Calculate(&hcrc, writeData, writeSize);
    // Crc should be sent in reverse
    writeData[writeSize] = (uint8_t) (crc >> 0);
    writeSize++;
    writeData[writeSize] = (uint8_t)  (crc >> 8);
    writeSize++;

    // Message should be ready here here

    BQ_SetMosiSPI(hbq); // Getting ready to transmit

    HAL_GPIO_WritePin(hbq->csGPIOx, hbq->csPin, GPIO_PIN_RESET);
    Align_DelayUs(10); // Safety margin
    HAL_SPI_Transmit(hbq->hspi, writeData, writeSize, BQ_TIMEOUT);
    Align_DelayUs(10); // Safety margin

    if(HAL_SPI_Receive(hbq->hspi, dataOut, dataLength, BQ_TIMEOUT) > 0){
        HAL_GPIO_WritePin(hbq->csGPIOx, hbq->csPin, GPIO_PIN_SET); // Cant forget to reset on an error
        BQ_SetMosiIdle(hbq); // Mosi always needs to be idle during end of command
        return 4;
    }

    HAL_GPIO_WritePin(hbq->csGPIOx, hbq->csPin, GPIO_PIN_SET);
    BQ_SetMosiIdle(hbq); // Mosi always needs to be idle during end of command
    Align_DelayUs(10); // Safety margin

    return 0;
}

// Output:
// 0 - Everything is ok
// 1 - SPI not ready, and timeout
// 2 - Invalid writeType
// 3 - Too much data, over 8 bytes, max supported by device
uint8_t BQ_Write(BQ_HandleTypeDef* hbq, uint8_t *inData, uint8_t deviceId, uint16_t regAddr, uint8_t dataLength, uint8_t writeType){
    uint32_t start = HAL_GetTick();
    while(BQ_SpiRdy(hbq) != true){
        if(HAL_GetTick() > start + BQ_TIMEOUT){
            return 1;
        }
    }
    if((writeType != BQ_DEVICE_WRITE) && (writeType != BQ_DEVICE_WRITE) && (writeType != BQ_DEVICE_WRITE)){
        return 2;
    }
    if(dataLength > 8){
        return 3;
    }
    // To limit the program size for now
    uint8_t writeData[12] = {0};

    uint8_t writeSize = 0;
    writeData[writeSize] = writeType;
    writeSize++;

    if(writeType == BQ_DEVICE_WRITE){
        writeData[writeSize] = deviceId;
        writeSize++;
    }
    writeData[writeSize] = (uint8_t) (regAddr >> 8);
    writeSize++;
    writeData[writeSize] = (uint8_t) regAddr;
    writeSize++;

    for(int i=0; i<dataLength; i++){
        writeData[writeSize] = inData[i];
        writeSize++;
    }
    
    uint16_t crc = HAL_CRC_Calculate(&hcrc, writeData, writeSize);
    writeData[writeSize] = (uint8_t) (crc >> 0);
    writeSize++;
    writeData[writeSize] = (uint8_t) (crc >> 8);
    writeSize++;
    
    // Whole transmit should be ready here
    BQ_SetMosiSPI(hbq); // Getting ready to transmit

    HAL_GPIO_WritePin(hbq->csGPIOx, hbq->csPin, GPIO_PIN_RESET);
    Align_DelayUs(1); // Atleast 50ns, we dont have that kind of resolution

    HAL_SPI_Transmit(hbq->hspi, writeData, writeSize, BQ_TIMEOUT);

    Align_DelayUs(10);
    HAL_GPIO_WritePin(hbq->csGPIOx, hbq->csPin, GPIO_PIN_SET);

    BQ_SetMosiIdle(hbq); // Mosi always needs to be idle during end of command


    return 0;
}