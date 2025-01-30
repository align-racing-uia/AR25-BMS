#include "bq79600.h"
#include "align-utils.h"
#include "string.h"
#include "spi.h"
#include "crc.h"
#include "math.h"

uint8_t bqOutputBuffer[128*TOTALBOARDS] = {0};
float bqCellVoltages[TOTAL_CELLS] = {0};
float bqDieTemperatures[2*(TOTALBOARDS-1)] = {0};


// We sometimes need full control of the MOSI Pin in GPIO between SPI commands
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

// As we quite often need GPIO control over the MOSI pin, we have to give it back once in a while as well
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

// Utility function to set the MOSI pin high, as defined in the datasheet of the BQ79600
void BQ_SetMosiIdle(BQ_HandleTypeDef* hbq){
    BQ_SetMosiGPIO(hbq);
    HAL_GPIO_WritePin(hbq->mosiGPIOx,hbq->mosiPin, GPIO_PIN_SET);
}

// Resets the communication FIFO on the BQ79600
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

// Wakes up the master from the shutdown state
void BQ_WakePing(BQ_HandleTypeDef* hbq){
    // We need to be able to control MOSI through GPIO during this
    BQ_SetMosiGPIO(hbq);

    HAL_GPIO_WritePin(hbq->csGPIOx, hbq->csPin, GPIO_PIN_RESET);
    Align_DelayUs(2); // Atleast 2 us

    HAL_GPIO_WritePin(hbq->mosiGPIOx, hbq->mosiPin, GPIO_PIN_RESET);
    Align_DelayUs(2500); // Atleast 2.5 ms
    HAL_GPIO_WritePin(hbq->mosiGPIOx, hbq->mosiPin, GPIO_PIN_SET);
    
    Align_DelayUs(2); // Atleast 2 us

    HAL_GPIO_WritePin(hbq->csGPIOx, hbq->csPin, GPIO_PIN_SET);

    HAL_Delay(4); // Atleast 3.5ms
}


// The master pings all chips on the stack to wake up from shutdown state
void BQ_WakeMsg(BQ_HandleTypeDef* hbq){

    // We need to be able to control MOSI through GPIO during this

    // Only normal thing that happens during this wake up call..
    uint8_t data[1] = {BQ_CONTROL1_SEND_WAKE}; // Data to send to CONTROL1[SEND_WAKE]
    BQ_Write(hbq, data, BQ_SELF_ID, BQ_CONTROL1, 1, BQ_DEVICE_WRITE);

    // Wait 1.6ms + 10ms pr device on bus
    // 10x 1.6+10 = 116ms
    
    HAL_Delay(120);
}

// Checks if the SPI Bus is ready for further communication
bool BQ_SpiRdy(BQ_HandleTypeDef* hbq){
    return HAL_GPIO_ReadPin(hbq->spiRdyGPIOx, hbq->spiRdyPin) == GPIO_PIN_SET;
}

// Auto addressing routine, used to create the stacks, and put all slaves on the stack
void BQ_AutoAddress(BQ_HandleTypeDef* hbq){

    uint8_t data[1] = {0};    
    for(int i=0; i<8; i++){ // Luckly all the registers are after eachother
        BQ_Write(hbq, data, BQ_SELF_ID, BQ_OTP_ECC_DATAIN1+i, 1, BQ_STACK_WRITE);
    }
    data[0] = BQ_CONTROL1_AA;
    BQ_Write(hbq, data, BQ_SELF_ID, BQ_CONTROL1, 1, BQ_BROAD_WRITE);
    for(int i=0; i<TOTALBOARDS; i++){
        data[0] = i;
        BQ_Write(hbq, data, BQ_SELF_ID, BQ_DIR0_ADDR, 1, BQ_BROAD_WRITE);
    }

    data[0] = 0x02;
    BQ_Write(hbq, data, BQ_SELF_ID, BQ_COMM_CTRL, 1, BQ_BROAD_WRITE);
    data[0] = 0x03;
    BQ_Write(hbq, data, TOTALBOARDS-1, BQ_COMM_CTRL, 1, BQ_DEVICE_WRITE);
    

    uint8_t readData[TOTALBOARDS] = {0};
    for(int i=0; i<8; i++){
        BQ_Read(hbq, readData, BQ_SELF_ID, BQ_OTP_ECC_DATAIN1+i, 1, BQ_STACK_READ);
    }
    
}

// Activates the ADC on all slaves
// Num of cells correspond to the number of cells in series each IC should measure (max 16)
void BQ_ActivateSlaveADC(BQ_HandleTypeDef* hbq){
    // Activate on the whole stack
    uint8_t data[1] = {CELLS_IN_SERIES - 6}; // 0x00 => 6 measured cells
    BQ_Write(hbq, data, 0, BQ16_ACTIVE_CELLS, 1, BQ_STACK_WRITE);
    data[0] = BQ16_ADC_CTRL1_ADCCONT | BQ16_ADC_CTRL1_MAINGO;
    BQ_Write(hbq, data, 0, BQ16_ADC_CTRL1, 1, BQ_BROAD_WRITE);
    // Wait for everyone to get the message
    Align_DelayUs(192 + (5*TOTALBOARDS));
}

// Reads ADC and converts them to voltages in place, and puts them on the global bqCellVoltages pointer, whose size should match the max cells variable
// TODO: Convert to a uint8_t return type, to define different error states, which can be handled properly
void BQ_GetCellVoltages(BQ_HandleTypeDef* hbq){
    // Cleanup
    memset(bqOutputBuffer, 0x00, BQ_OUTPUT_BUFFER_SIZE);
    memset(bqCellVoltages, 0x00, TOTAL_CELLS*sizeof(float));

    BQ_Read(hbq, bqOutputBuffer, 0, BQ16_VCELL16_HI +( 2*(16-CELLS_IN_SERIES)), CELLS_IN_SERIES*2, BQ_STACK_READ); // 2 registers for each cell

    uint8_t totalLen = 6 + CELLS_IN_SERIES * 2; // Totalt expected message length

    for(uint8_t i=0;i<TOTALBOARDS-1;i++){ // Base board will not be part of the cell voltages

        // For now, ignore all CRC checking and verifications, we want the data
        // TODO: Implement proper CRC verification

        // The responses are always:
        // 1 bytes for message length (minus 1), 1 byte for device id, 2 bytes for register, data inbetween, 2 bytes for CRC

        uint8_t len = bqOutputBuffer[i*totalLen]+1; // Should be known, but might as well

        for(uint8_t y=0; y<len; y+=2){
            uint16_t rawAdc = (((uint16_t) bqOutputBuffer[i*totalLen+4+y]) << 8) | ((uint16_t) bqOutputBuffer[i*totalLen+5+y]);
            bqCellVoltages[CELLS_IN_SERIES*i+y/2] = (float) ((float) rawAdc * 0.00019073); // in mV
        }



    }
}

void BQ_GetDieTemperature(BQ_HandleTypeDef* hbq){
    // Cleanup
    memset(bqOutputBuffer, 0x00, BQ_OUTPUT_BUFFER_SIZE);
    memset(bqDieTemperatures, 0x00, 2*(TOTALBOARDS-1));

    BQ_Read(hbq, bqOutputBuffer, 0, BQ16_DIETEMP1_HI, 2, BQ_STACK_READ);

    uint8_t totalLen = 8; // pr board
    for(uint8_t i=0; i<TOTALBOARDS-1; i++){

        // For now, ignore all CRC checking and verifications, we want the data
        // TODO: Implement proper CRC verification

        uint16_t rawTemp = ((uint16_t)(bqOutputBuffer[i*totalLen+4] << 8)) | ((uint16_t)(bqOutputBuffer[i*totalLen+4]));
        // As of now, we are only getting the temperature of Die 1
        bqDieTemperatures[2*i] = rawTemp * 0.025; // degrees Celcius

    }

}




// Should be used with bqOutputBuffer
// Output:
// 0 - Everything is ok
// 1 - SPI not ready, and timeout
// 2 - Invalid read type
// 3 - Asking for too much data (max 128 bytes) or too little (under 1)
// 4 - Recieve timeout
uint8_t BQ_Read(BQ_HandleTypeDef* hbq, uint8_t *pOut, uint8_t deviceId, uint16_t regAddr, uint8_t dataLength, uint8_t readType){
    uint32_t start = HAL_GetTick();
    while(BQ_SpiRdy(hbq) != true){
        if(HAL_GetTick() > start + BQ_TIMEOUT){
            return 1;
        }
    }
    if((readType != BQ_DEVICE_READ) && (readType != BQ_STACK_READ) && (readType != BQ_BROAD_READ)){
        return 2;
    }

    if(dataLength > 128 || dataLength < 1){
        return 3;
    }

    // Formatting message
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
    writeData[writeSize] = dataLength-1;
    writeSize++;
    uint16_t crc = HAL_CRC_Calculate(&hcrc, writeData, writeSize);
    // Crc should be sent in reverse
    writeData[writeSize] = (uint8_t) (crc >> 0);
    writeSize++;
    writeData[writeSize] = (uint8_t)  (crc >> 8);
    writeSize++;

    // Message should be ready here here

    BQ_SetMosiSPI(hbq); // Getting ready to transmit

    // Transmitting message
    HAL_GPIO_WritePin(hbq->csGPIOx, hbq->csPin, GPIO_PIN_RESET);
    Align_DelayUs(10); // Safety margin
    HAL_SPI_Transmit(hbq->hspi, writeData, writeSize, BQ_TIMEOUT);
    Align_DelayUs(10); // Safety margin
    HAL_GPIO_WritePin(hbq->csGPIOx, hbq->csPin, GPIO_PIN_SET);
    

    // How many bytes do we expect?
    uint16_t maxBytes = 0;

    if(readType == BQ_DEVICE_READ){
        maxBytes = dataLength + 6;
    }else if(readType == BQ_STACK_READ){
        maxBytes = (dataLength + 6) * (TOTALBOARDS - 1);
    }else if(readType == BQ_BROAD_READ){
        maxBytes = (dataLength + 6) * (TOTALBOARDS);
    }

    uint16_t fullBuffers = (uint16_t) (maxBytes/128);
    uint16_t remainingBytes = maxBytes - (fullBuffers*128);
    BQ_SetMosiIdle(hbq); // We need to transmit 0xFFs.... so why not...?
    while(remainingBytes>0){
        start = HAL_GetTick();
        while(BQ_SpiRdy(hbq) != true){
            if(HAL_GetTick() > start + BQ_TIMEOUT){
                BQ_SetMosiIdle(hbq); // Lets not make the issue bigger than it is
                BQ_ClearComm(hbq);
                return 4;
            }
        }

        if(fullBuffers>0){
            HAL_GPIO_WritePin(hbq->csGPIOx, hbq->csPin, GPIO_PIN_RESET);
            Align_DelayUs(10); // Safety margin
            HAL_SPI_Receive(hbq->hspi, pOut, 128, BQ_TIMEOUT);
            Align_DelayUs(10); // Safety margin
            HAL_GPIO_WritePin(hbq->csGPIOx, hbq->csPin, GPIO_PIN_SET);
            pOut += 128; // Move pointer 128 steps
            fullBuffers--;
        }else{
            HAL_GPIO_WritePin(hbq->csGPIOx, hbq->csPin, GPIO_PIN_RESET);
            Align_DelayUs(10); // Safety margin
            HAL_SPI_Receive(hbq->hspi, pOut, remainingBytes, BQ_TIMEOUT);
            Align_DelayUs(10); // Safety margin
            HAL_GPIO_WritePin(hbq->csGPIOx, hbq->csPin, GPIO_PIN_SET);
            remainingBytes = 0;
        }
    }
    // All data should now be in pOut location (most ofte bqOutputBuffer)

    BQ_SetMosiIdle(hbq); // Mosi always needs to be idle during end of command

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
    if((writeType != BQ_DEVICE_WRITE) && (writeType != BQ_STACK_WRITE) && (writeType != BQ_BROAD_WRITE)){
        return 2;
    }
    if(dataLength > 8 || dataLength < 1){
        return 3;
    }
    // To limit the program size for now
    uint8_t writeData[12] = {0};

    uint8_t writeSize = 0;
    writeData[writeSize] = writeType | (dataLength-1);    
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
    Align_DelayUs(10); // Atleast 50ns, we dont have that kind of resolution

    HAL_SPI_Transmit(hbq->hspi, writeData, writeSize, BQ_TIMEOUT);

    Align_DelayUs(10);
    HAL_GPIO_WritePin(hbq->csGPIOx, hbq->csPin, GPIO_PIN_SET);

    BQ_SetMosiIdle(hbq); // Mosi always needs to be idle during end of command


    return 0;
}