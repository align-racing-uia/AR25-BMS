#include "bq79600.h"
#include "alignutils.h"
#include "string.h"
#include "spi.h"
#include "crc.h"
#include "math.h"
#include "stdlib.h"

void BQ_AllocateMemory(BQ_HandleTypeDef* hbq){
    // Allocate memory for the output buffer
    // For not enough memory allocation errors, we could return a status type, but this is such a substantial error that we just want to stop the program
    hbq->bqOutputBuffer = (uint8_t*)malloc(128*(hbq->bms_config->NumOfBoards));
    if(hbq->bqOutputBuffer == NULL){
        // Handle memory allocation error
        Error_Handler();
    }

    hbq->cellVoltages = (float*)malloc((hbq->bms_config->CellCount)*sizeof(float));
    if(hbq->cellVoltages == NULL){
        // Handle memory allocation error
        Error_Handler();
    }
    hbq->bqDieTemperatures = (float*)malloc((2*(hbq->bms_config->NumOfBoards-1))*sizeof(float));
    if(hbq->bqDieTemperatures == NULL){
        // Handle memory allocation error
        Error_Handler();
    }
    // Set to default values
    for(int i=0; i<128*(hbq->bms_config->NumOfBoards); i++){
        hbq->bqOutputBuffer[i] = 0;
    }
    for(int i=0; i<hbq->bms_config->CellCount; i++){
        hbq->cellVoltages[i] = 0;
    }
    for(int i=0; i<2*(hbq->bms_config->NumOfBoards-1); i++){
        hbq->bqDieTemperatures[i] = 0;
    }
}

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
BQ_StatusTypeDef BQ_WakeMsg(BQ_HandleTypeDef* hbq){

    // We need to be able to control MOSI through GPIO during this

    // Only normal thing that happens during this wake up call..
    uint8_t data[1] = {BQ_CONTROL1_SEND_WAKE}; // Data to send to CONTROL1[SEND_WAKE]
    BQ_StatusTypeDef status = BQ_Write(hbq, data, BQ_SELF_ID, BQ_CONTROL1, 1, BQ_DEVICE_WRITE);

    // Wait 1.6ms + 10ms pr device on bus
    // 10x 1.6+10 = 116ms
    
    HAL_Delay(120);
    return status;
}

// Checks if the SPI Bus is ready for further communication
bool BQ_SpiRdy(BQ_HandleTypeDef* hbq){
    return HAL_GPIO_ReadPin(hbq->spiRdyGPIOx, hbq->spiRdyPin) == GPIO_PIN_SET;
}

// Auto addressing routine, used to create the stacks, and put all slaves on the stack
BQ_StatusTypeDef BQ_AutoAddress(BQ_HandleTypeDef* hbq){

    // Relatively undocumented as its more or less a straight copy of what the TI driver does
    uint8_t data[1] = {0};
    BQ_StatusTypeDef status;    
    for(int i=0; i<8; i++){ // Luckly all the registers are after eachother
        status = BQ_Write(hbq, data, BQ_SELF_ID, BQ_OTP_ECC_DATAIN1+i, 1, BQ_STACK_WRITE);
        if(status != BQ_STATUS_OK){
            return status;
        }
    }
    data[0] = BQ_CONTROL1_AA;
    status = BQ_Write(hbq, data, BQ_SELF_ID, BQ_CONTROL1, 1, BQ_BROAD_WRITE);
    if(status != BQ_STATUS_OK){
        return status;
    }
    for(int i=0; i<hbq->bms_config->NumOfBoards; i++){
        data[0] = i;
        status = BQ_Write(hbq, data, BQ_SELF_ID, BQ_DIR0_ADDR, 1, BQ_BROAD_WRITE);
        if(status != BQ_STATUS_OK){
            return status;
        }
    }

    data[0] = 0x02;
    status = BQ_Write(hbq, data, BQ_SELF_ID, BQ_COMM_CTRL, 1, BQ_BROAD_WRITE);
    data[0] = 0x03;
    status = BQ_Write(hbq, data, hbq->bms_config->NumOfBoards-1, BQ_COMM_CTRL, 1, BQ_DEVICE_WRITE);
    if(status != BQ_STATUS_OK){ // The odds of one failing but not the other is very small, as the only possible error would be a SPI fault
        return status;
    }

    memset(hbq->bqOutputBuffer, 0x00, 128*(hbq->bms_config->NumOfBoards)); // Clear the output buffer
    for(int i=0; i<8; i++){
        status = BQ_Read(hbq, hbq->bms_config->NumOfBoards, BQ_SELF_ID, BQ_OTP_ECC_DATAIN1+i, 1, BQ_STACK_READ);
        if(status != BQ_STATUS_OK){
            return status;
        }
    }
    return status;
    
}

// Activates the main ADC on all slaves
// Num of cells correspond to the number of cells in series each IC should measure (max 16)
BQ_StatusTypeDef BQ_ActivateSlaveADC(BQ_HandleTypeDef* hbq){
    // Activate on the whole stack
    uint8_t data = hbq->bms_config->CellCountInSeries - 6; // 0x00 => 6 measured cells
    BQ_StatusTypeDef status;
    status = BQ_Write(hbq, &data, 0, BQ16_ACTIVE_CELLS, 1, BQ_STACK_WRITE);
    if(status != BQ_STATUS_OK){
        return status;
    }
    data = BQ16_ADC_CTRL1_ADCCONT | BQ16_ADC_CTRL1_MAINGO;
    status = BQ_Write(hbq, &data, 0, BQ16_ADC_CTRL1, 1, BQ_BROAD_WRITE);
    // Wait for everyone to get the message
    Align_DelayUs(192 + (5*hbq->bms_config->NumOfBoards));
    return status;
}

// This sets the selected GPIO of all the slaves the GPIO are 0 indexed
BQ_StatusTypeDef BQ_SetGPIOAll(BQ_HandleTypeDef* hbq, uint8_t pin, bool logicState){

    uint8_t data[1] = {0};
    uint8_t register_offset = pin / 2;
    uint8_t pin_offset = pin % 2;

    // I really wish there was an atomic write for this.. But we need to rely on the local data instead
    if(logicState){
        hbq->gpioconf[register_offset] |= (1 << (pin_offset*3));
    }else{
        hbq->gpioconf[register_offset] &= ~(1 << (pin_offset*3));
    }


    BQ_StatusTypeDef status = BQ_Write(hbq, &(hbq->gpioconf[register_offset]), 0, BQ16_GPIO1_HI + register_offset, 1, BQ_BROAD_WRITE);
    // Wait for everyone to get the message
    Align_DelayUs(192 + (5*hbq->bms_config->NumOfBoards));
    return status;

}

// This function configures the GPIOs of the slaves to be either ADCs or GPIOs, based on the gpioADC variable
BQ_StatusTypeDef BQ_ConfigureGPIO(BQ_HandleTypeDef* hbq){

    BQ_StatusTypeDef status;
    for(int i=0; i<4; i++){
        // Look at this amazing mess
        uint8_t data[1] = {0};
        for(int j=0; j<2; j++){
            if(hbq->gpioADC >> (j+i*2) & 0x01){
                if(j == 0){
                    data[0] |= BQ16_GPIO_CONF1_GPIO1_ADC;
                }else{
                    data[0] |= BQ16_GPIO_CONF1_GPIO2_ADC;
                }
            }else{
                if(j == 0){
                    data[0] |= BQ16_GPIO_CONF1_GPIO1_OUTPUT;
                }else{
                    data[0] |= BQ16_GPIO_CONF1_GPIO2_OUTPUT;
                }
            }
        }

        status = BQ_Write(hbq, data, 0, BQ16_GPIO_CONF1+i, 1, BQ_STACK_WRITE);
        // Wait for everyone to get the message
        Align_DelayUs(192 + (5*hbq->bms_config->NumOfBoards));
        if(status != BQ_STATUS_OK){
            return status;
        }
        hbq->gpioconf[i] = data[0]; // Save the config for later
    }

    uint8_t data[1] = {BQ16_ADC_CTRL3_AUXCONT | BQ16_ADC_CTRL3_AUXGO};
    status = BQ_Write(hbq, data, 0, BQ16_ADC_CTRL3, 1, BQ_STACK_WRITE);
    // Wait for everyone to get the message
    Align_DelayUs(192 + (5*hbq->bms_config->NumOfBoards));
    return status;

}

// Reads ADC and converts them to voltages in place, and puts them on the global bqCellVoltages pointer, whose size should match the max cells variable
// TODO: Convert to a uint8_t return type, to define different error states, which can be handled properly
BQ_StatusTypeDef BQ_GetCellVoltages(BQ_HandleTypeDef* hbq){
    // Cleanup
    memset(hbq->bqOutputBuffer, 0x00, 128*(hbq->bms_config->NumOfBoards));
    memset(hbq->cellVoltages, 0x00, hbq->bms_config->CellCount*sizeof(float));
    BQ_StatusTypeDef status;
    status = BQ_Read(hbq, hbq->bqOutputBuffer, 0, BQ16_VCELL16_HI +( 2*(16-hbq->bms_config->CellCountInSeries)), hbq->bms_config->CellCountInSeries*2, BQ_STACK_READ); // 2 registers for each cell

    if(status != BQ_STATUS_OK){
        return status;
    }

    uint8_t totalLen = 6 + hbq->bms_config->CellCountInSeries * 2; // Totalt expected message length

    for(uint8_t i=0;i<hbq->bms_config->NumOfBoards-1;i++){ // Base board will not be part of the cell voltages

        // For now, ignore all CRC checking and verifications, we want the data
        // TODO: Implement proper CRC verification

        // The responses are always:
        // 1 bytes for message length (minus 1), 1 byte for device id, 2 bytes for register, data inbetween, 2 bytes for CRC

        uint8_t len = hbq->bqOutputBuffer[i*totalLen]+1; // Should be known, but might as well

        for(uint8_t y=0; y<len; y+=2){
            uint16_t rawAdc = (((uint16_t) hbq->bqOutputBuffer[i*totalLen+4+y]) << 8) | ((uint16_t) hbq->bqOutputBuffer[i*totalLen+5+y]);
            hbq->cellVoltages[hbq->bms_config->CellCountInSeries*i+y/2] = (float) ((float) rawAdc * 0.00019073); // in mV
        }



    }
    return status;
}

BQ_StatusTypeDef BQ_GetCellTemperatures(BQ_HandleTypeDef* hbq){

    BQ_SetGPIOAll(hbq, 7, true); // Set GPIO8 to high

    // TODO Implement temperature reading

    BQ_SetGPIOAll(hbq, 8, false); // Set GPIO8 to low

    // TODO Implement temperature reading


    return BQ_STATUS_OK;
}


BQ_StatusTypeDef BQ_GetDieTemperature(BQ_HandleTypeDef* hbq){
    // Cleanup
    memset(hbq->bqOutputBuffer, 0x00, 128*(hbq->bms_config->NumOfBoards));
    memset(hbq->bqDieTemperatures, 0x00, 2*(hbq->bms_config->NumOfBoards-1)*sizeof(float));
    BQ_StatusTypeDef status = BQ_STATUS_OK;
    status = BQ_Read(hbq, hbq->bqOutputBuffer, 0, BQ16_DIETEMP1_HI, 2, BQ_STACK_READ);

    if(status != BQ_STATUS_OK){
        return status;
    }
    uint8_t totalLen = 8; // pr board
    for(uint8_t i=0; i<hbq->bms_config->NumOfBoards-1; i++){

        // For now, ignore all CRC checking and verifications, we want the data
        // TODO: Implement proper CRC verification

        uint16_t rawTemp = ((uint16_t)(hbq->bqOutputBuffer[i*totalLen+4] << 8)) | ((uint16_t)(hbq->bqOutputBuffer[i*totalLen+4]));
        // As of now, we are only getting the temperature of Die 1
        hbq->bqDieTemperatures[2*i] = rawTemp * 0.025; // degrees Celcius

    }

    memset(hbq->bqOutputBuffer, 0x00, 128*(hbq->bms_config->NumOfBoards));

    return BQ_Read(hbq, hbq->bqOutputBuffer, 0, BQ16_DIETEMP2_HI, 2, BQ_STACK_READ);

}




// Should be used with bqOutputBuffer
// Output:
// 0 - Everything is ok
// 1 - SPI not ready, and timeout
// 2 - Invalid read type
// 3 - Asking for too much data (max 128 bytes) or too little (under 1)
// 4 - Recieve timeout
BQ_StatusTypeDef BQ_Read(BQ_HandleTypeDef* hbq, uint8_t *pOut, uint8_t deviceId, uint16_t regAddr, uint8_t dataLength, uint8_t readType){
    uint32_t start = HAL_GetTick();
    while(BQ_SpiRdy(hbq) != true){
        if(HAL_GetTick() > start + BQ_TIMEOUT){
            return BQ_STATUS_TIMEOUT;
        }
    }
    if((readType != BQ_DEVICE_READ) && (readType != BQ_STACK_READ) && (readType != BQ_BROAD_READ)){
        return BQ_STATUS_DATA_ERROR;
    }

    if(dataLength > 128 || dataLength < 1){
        return BQ_STATUS_DATA_ERROR;
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
        maxBytes = (dataLength + 6) * (hbq->bms_config->NumOfBoards - 1);
    }else if(readType == BQ_BROAD_READ){
        maxBytes = (dataLength + 6) * (hbq->bms_config->NumOfBoards);
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
                return BQ_STATUS_TIMEOUT;
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

    return BQ_STATUS_OK;
}

// Output:
// 0 - Everything is ok
// 1 - SPI not ready, and timeout
// 2 - Invalid writeType
// 3 - Too much data, over 8 bytes, max supported by device
BQ_StatusTypeDef BQ_Write(BQ_HandleTypeDef* hbq, uint8_t *inData, uint8_t deviceId, uint16_t regAddr, uint8_t dataLength, uint8_t writeType){
    uint32_t start = HAL_GetTick();
    while(BQ_SpiRdy(hbq) != true){
        if(HAL_GetTick() > start + BQ_TIMEOUT){
            return BQ_STATUS_TIMEOUT;
        }
    }
    if((writeType != BQ_DEVICE_WRITE) && (writeType != BQ_STACK_WRITE) && (writeType != BQ_BROAD_WRITE)){
        return BQ_STATUS_DATA_ERROR;
    }
    if(dataLength > 8 || dataLength < 1){
        return BQ_STATUS_DATA_ERROR;
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


    return BQ_STATUS_OK;
}