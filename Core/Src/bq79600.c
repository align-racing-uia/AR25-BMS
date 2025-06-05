#include "bq79600.h"
#include "alignutils.h"
#include "string.h"
#include "spi.h"
#include "crc.h"
#include "math.h"
#include "stdlib.h"

// Private helper function

uint8_t reverse_bits(uint8_t b)
{
    b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
    b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
    b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
    return b;
}

// Depends on the GCC compiler
#define NUM_OF_ONES(x) __builtin_popcount(x)

void BQ_Init(BQ_HandleTypeDef *hbq)
{
    // Set the GPIOs to their default state
    HAL_GPIO_WritePin(hbq->CsPin.GPIOx, hbq->CsPin.Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(hbq->MosiPin.GPIOx, hbq->MosiPin.Pin, GPIO_PIN_RESET);
    hbq->HighestCellTemperature = 0.0f;
    hbq->LowestCellTemperature = 0.0f; // Set the lowest cell temperature to a high value

}

void BQ_Configure(BQ_HandleTypeDef *hbq, BQ_ConfigTypeDef *bq_config)
{
    if (bq_config == NULL || hbq == NULL)
    {
        // If this occurs, you have done something very wrong
        Error_Handler();
    }

    hbq->NumOfSlaves = bq_config->NumOfSlaves;
    hbq->NumOfChips = bq_config->NumOfSlaves + 1; // The master chip is always present
    hbq->NumOfCellsEach = bq_config->NumOfCellsEach;
    
    hbq->TempMultiplexEnabled = bq_config->TempMultiplexEnabled;
    hbq->TempMultiplexPinIndex = bq_config->TempMultiplexPinIndex;
    hbq->GpioAuxADCMap = bq_config->GpioAuxADCMap;
    hbq->CellTempPinMap = bq_config->CellTempPinMap;
}

void BQ_BindHardware(BQ_HandleTypeDef *hbq, SPI_HandleTypeDef *hspi, BQ_PinTypeDef cs_pin, BQ_PinTypeDef spi_rdy_pin, BQ_PinTypeDef mosi_pin, BQ_PinTypeDef fault_pin, TIM_HandleTypeDef *htim)
{
    if (hbq == NULL || hspi == NULL || htim == NULL)
    {
        // If this occurs, youve done something very wrong
        Error_Handler();
    }

    hbq->hspi = hspi;
    hbq->CsPin = cs_pin;
    hbq->SpiRdyPin = spi_rdy_pin;
    hbq->MosiPin = mosi_pin;
    hbq->FaultPin = fault_pin;
    hbq->htim = htim;
}

// The use of floats here is to be able to take advantage of the FPU, and to make the maths in later stages simpler
void BQ_BindMemory(BQ_HandleTypeDef *hbq, uint8_t *bq_output_buffer, float *cell_voltages_memory_pool, float *cell_temperature_memory_pool, float *bq_die_temperature_memory_pool)
{
    if (hbq == NULL)
    {
        // If this occurs, youve done something very wrong
        Error_Handler();
    }

    hbq->BQOutputBuffer = bq_output_buffer;
    if (hbq->BQOutputBuffer == NULL)
    {
        // Handle memory allocation error
        Error_Handler();
    }

    hbq->CellVoltages = cell_voltages_memory_pool;
    if (hbq->CellVoltages == NULL)
    {
        // Handle memory allocation error
        Error_Handler();
    }
    hbq->BQDieTemperatures = bq_die_temperature_memory_pool;
    if (hbq->BQDieTemperatures == NULL)
    {
        // Handle memory allocation error
        Error_Handler();
    }
    hbq->CellTemperatures = cell_temperature_memory_pool;
    if (hbq->CellTemperatures == NULL)
    {
        // Handle memory allocation error
        Error_Handler();
    }
}

// We sometimes need full control of the MOSI Pin in GPIO between SPI commands
void BQ_SetMosiGPIO(BQ_HandleTypeDef *hbq)
{

    HAL_GPIO_DeInit(hbq->MosiPin.GPIOx, hbq->MosiPin.Pin);
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = hbq->MosiPin.Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(hbq->MosiPin.GPIOx, &GPIO_InitStruct);
}

// As we quite often need GPIO control over the MOSI pin, we have to give it back once in a while as well
void BQ_SetMosiSPI(BQ_HandleTypeDef *hbq)
{

    HAL_GPIO_DeInit(hbq->MosiPin.GPIOx, hbq->MosiPin.Pin);
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = hbq->MosiPin.Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(hbq->MosiPin.GPIOx, &GPIO_InitStruct);
}

// Utility function to set the MOSI pin high, as defined in the datasheet of the BQ79600
void BQ_SetMosiIdle(BQ_HandleTypeDef *hbq)
{
    BQ_SetMosiGPIO(hbq);
    HAL_GPIO_WritePin(hbq->MosiPin.GPIOx, hbq->MosiPin.Pin, GPIO_PIN_SET);
}

// Resets the communication FIFO on the BQ79600
void BQ_ClearComm(BQ_HandleTypeDef *hbq)
{
    BQ_SetMosiSPI(hbq);

    HAL_GPIO_WritePin(hbq->CsPin.GPIOx, hbq->CsPin.Pin, GPIO_PIN_RESET);
    Align_DelayUs(hbq->htim, 100);
    uint8_t data[1] = {0};
    HAL_SPI_Transmit(hbq->hspi, data, 1, BQ_TIMEOUT);
    Align_DelayUs(hbq->htim, 100);
    HAL_GPIO_WritePin(hbq->CsPin.GPIOx, hbq->CsPin.Pin, GPIO_PIN_SET);

    BQ_SetMosiIdle(hbq); // Should always set Mosi Idle when not sending commands
}

// Wakes up the master from the shutdown state
void BQ_WakePing(BQ_HandleTypeDef *hbq)
{
    // We need to be able to control MOSI through GPIO during this
    BQ_SetMosiGPIO(hbq);

    HAL_GPIO_WritePin(hbq->CsPin.GPIOx, hbq->CsPin.Pin, GPIO_PIN_RESET);
    Align_DelayUs(hbq->htim, 2); // Atleast 2 us

    HAL_GPIO_WritePin(hbq->MosiPin.GPIOx, hbq->MosiPin.Pin, GPIO_PIN_RESET);
    Align_DelayUs(hbq->htim, 2500); // Atleast 2.5 ms
    HAL_GPIO_WritePin(hbq->MosiPin.GPIOx, hbq->MosiPin.Pin, GPIO_PIN_SET);

    Align_DelayUs(hbq->htim, 2); // Atleast 2 us

    HAL_GPIO_WritePin(hbq->CsPin.GPIOx, hbq->CsPin.Pin, GPIO_PIN_SET);

    HAL_Delay(4); // Atleast 3.5ms
}

// The master pings all chips on the stack to wake up from shutdown state
BQ_StatusTypeDef BQ_WakeMsg(BQ_HandleTypeDef *hbq)
{

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
bool BQ_SpiRdy(BQ_HandleTypeDef *hbq)
{
    return HAL_GPIO_ReadPin(hbq->SpiRdyPin.GPIOx, hbq->SpiRdyPin.Pin) == GPIO_PIN_SET;
}

// Auto addressing routine, used to create the stacks, and put all slaves on the stack
BQ_StatusTypeDef BQ_AutoAddress(BQ_HandleTypeDef *hbq)
{

    // Relatively undocumented as its more or less a straight copy of what the TI driver does
    uint8_t data[1] = {0};
    BQ_StatusTypeDef status;
    for (int i = 0; i < 8; i++)
    { // Luckly all the registers are after eachother
        status = BQ_Write(hbq, data, BQ_SELF_ID, BQ_OTP_ECC_DATAIN1 + i, 1, BQ_STACK_WRITE);
        if (status != BQ_STATUS_OK)
        {
            return status;
        }
    }
    data[0] = BQ_CONTROL1_AA;
    status = BQ_Write(hbq, data, BQ_SELF_ID, BQ_CONTROL1, 1, BQ_BROAD_WRITE);
    if (status != BQ_STATUS_OK)
    {
        return status;
    }
    for (int i = 0; i < hbq->NumOfChips; i++)
    {
        data[0] = i;
        status = BQ_Write(hbq, data, BQ_SELF_ID, BQ_DIR0_ADDR, 1, BQ_BROAD_WRITE);
        if (status != BQ_STATUS_OK)
        {
            return status;
        }
    }

    data[0] = 0x02;
    status = BQ_Write(hbq, data, BQ_SELF_ID, BQ_COMM_CTRL, 1, BQ_BROAD_WRITE);
    data[0] = 0x03;
    status = BQ_Write(hbq, data, hbq->NumOfSlaves, BQ_COMM_CTRL, 1, BQ_DEVICE_WRITE);
    if (status != BQ_STATUS_OK)
    { // The odds of one failing but not the other is very small, as the only possible error would be a SPI fault
        return status;
    }

    memset(hbq->BQOutputBuffer, 0x00, 128 * (hbq->NumOfChips)); // Clear the output buffer
    for (int i = 0; i < 8; i++)
    {
        status = BQ_Read(hbq, hbq->BQOutputBuffer, BQ_SELF_ID, BQ_OTP_ECC_DATAIN1 + i, 1, BQ_STACK_READ);
        if (status != BQ_STATUS_OK)
        {
            return status;
        }
    }
    return status;
}

BQ_StatusTypeDef BQ_EnableCommTimeout(BQ_HandleTypeDef *hbq){
    if (hbq == NULL)
    {
        // If this occurs, youve done something very wrong
        Error_Handler();
    }

    // Set the timeout duration for loss of communication (Which occurs)
    // We default to 10s for the long comm timeout
    uint8_t data = 0b00001011; // 10 seconds, and shutdown on long comm timeout
    return BQ_Write(hbq, &data, BQ_SELF_ID, BQ16_COMM_TIMEOUT_CONF, 1, BQ_STACK_WRITE);
}

BQ_StatusTypeDef BQ_EnableTsRef(BQ_HandleTypeDef *hbq)
{
    if (hbq == NULL)
    {
        // If this occurs, youve done something very wrong
        Error_Handler();
    }

    // Enable the TSREF pin on all slaves
    uint8_t data = 0b00000001; // Enable TSREF pin
    return BQ_Write(hbq, &data, BQ_SELF_ID, BQ16_CONTROL2, 1, BQ_STACK_WRITE);
}



// Activates the main ADC on all slaves
// Num of cells correspond to the number of cells in series each IC should measure (max 16)
BQ_StatusTypeDef BQ_ActivateSlaveADC(BQ_HandleTypeDef *hbq)
{
    // Activate on the whole stack
    uint8_t data = hbq->NumOfCellsEach - 6; // 0x00 => 6 measured cells
    BQ_StatusTypeDef status;
    status = BQ_Write(hbq, &data, 0, BQ16_ACTIVE_CELLS, 1, BQ_STACK_WRITE);
    if (status != BQ_STATUS_OK)
    {
        return status;
    }
    data = BQ16_ADC_CTRL1_ADCCONT | BQ16_ADC_CTRL1_MAINGO;
    status = BQ_Write(hbq, &data, 0, BQ16_ADC_CTRL1, 1, BQ_STACK_WRITE);
    // Wait for everyone to get the message
    Align_DelayUs(hbq->htim, 192 + (5 * hbq->NumOfChips));
    return status;
}

// Activates the auxilery ADC on all slaves
BQ_StatusTypeDef BQ_ActivateAuxADC(BQ_HandleTypeDef *hbq)
{

    uint8_t data[1] = {BQ16_ADC_CTRL3_AUXCONT | BQ16_ADC_CTRL3_AUXGO};
    BQ_StatusTypeDef status = BQ_Write(hbq, data, 0, BQ16_ADC_CTRL3, 1, BQ_STACK_WRITE);
    // Wait for everyone to get the message
    Align_DelayUs(hbq->htim, 192 + (5 * hbq->NumOfChips));
    return status;
}

// Function to read the Auxilery ADCs of the slaves
BQ_StatusTypeDef BQ_GetGpioMeasurements(BQ_HandleTypeDef *hbq, uint8_t pin_map, uint8_t *data_out)
{
    // The pin map is a bitwise select of what GPIOs should be read as ADCs
    //    GPIO8 GPIO7 GPIO6 ...
    //  0b  0     0     0   ...

    // TODO: Implement proper CRC verification
    uint8_t output_buffer[8*2*hbq->NumOfSlaves]; // A maximum of 8 GPIOs, each with 2 bytes of data each, for each slave
    size_t memory_offset = 0;
    size_t total_len = 8; // For each message
    for (int i = 0; i < 8; i++)
    {
        if ((pin_map >> i) & 0x01)
        {

            BQ_StatusTypeDef status = BQ_Read(hbq, output_buffer, 0, BQ16_GPIO1_HI + i * 2, 2, BQ_STACK_READ); // Read the GPIO configuration register
            memory_offset += total_len;                                                                                              // 2 bytes for the GPIO data, 6 bytes for the header
            if (status != BQ_STATUS_OK)
            {
                return status;
            }
        }
    }
    size_t number_of_pins = NUM_OF_ONES(pin_map); // Number of pins to read
    for (size_t i = 0; i < number_of_pins; i++)
    {
        data_out[2 * i] = output_buffer[total_len * i - 2];
        data_out[2 * i + 1] = output_buffer[total_len * i - 1]; // Put data in the output buffer
    }

    return BQ_STATUS_OK;
}

BQ_StatusTypeDef BQ_GetTsRefMeasurements(BQ_HandleTypeDef *hbq, uint8_t *data_out)
{
    // The TSREF pin is used to measure the temperature of the BQ79600 chip itself
    // It is a single ADC channel, so we only need to read one register
    // The data is 2 bytes long, and we will return it in the data_out buffer

    BQ_StatusTypeDef status = BQ_Read(hbq, data_out, 0, BQ16_TSREF_HI, 2, BQ_STACK_READ);
    if (status != BQ_STATUS_OK)
    {
        return status;
    }

    return status;
}

// This sets the selected GPIO of all the slaves the GPIO are 0 indexed
BQ_StatusTypeDef BQ_SetGPIOAll(BQ_HandleTypeDef *hbq, uint8_t pin, bool logic_state)
{

    uint8_t data[1] = {0};
    uint8_t pin_offset = pin % 2;
    uint8_t register_offset = pin / 2;

    // I really wish there was an atomic write for this.. But we need to rely on the local data instead
    hbq->GpioConf[register_offset] |= (1 << (pin_offset * 3 + 2));  // Set the GPIO to output
    hbq->GpioConf[register_offset] &= ~(1 << (pin_offset * 3 + 1)); // Set the GPIO to output
    if (logic_state)
    {
        hbq->GpioConf[register_offset] |= (1 << (pin_offset * 3));
    }
    else
    {
        hbq->GpioConf[register_offset] &= ~(1 << (pin_offset * 3));
    }

    BQ_StatusTypeDef status = BQ_Write(hbq, &(hbq->GpioConf[register_offset]), 0, BQ16_GPIO_CONF1 + register_offset, 1, BQ_STACK_WRITE);
    // Wait for everyone to get the message
    Align_DelayUs(hbq->htim, 192 + (5 * hbq->NumOfChips));
    return status;
}

// This function configures the GPIOs of the slaves to be either ADCs or GPIOs, based on the GpioAuxADCMap variable
BQ_StatusTypeDef BQ_ConfigureGPIO(BQ_HandleTypeDef *hbq)
{

    BQ_StatusTypeDef status;
    for (int i = 0; i < 4; i++)
    {
        // Look at this amazing mess
        uint8_t data[1] = {0};
        for (int j = 0; j < 2; j++)
        {
            if (hbq->GpioAuxADCMap >> (j + i * 2) & 0x01)
            {
                if (j == 0)
                {
                    data[0] |= BQ16_GPIO_CONF1_GPIO1_ADC;
                }
                else
                {
                    data[0] |= BQ16_GPIO_CONF1_GPIO2_ADC;
                }
            }
            else
            {
                if (j == 0)
                {
                    data[0] |= BQ16_GPIO_CONF1_GPIO1_OUTPUT;
                }
                else
                {
                    data[0] |= BQ16_GPIO_CONF1_GPIO2_OUTPUT;
                }
            }
        }

        status = BQ_Write(hbq, data, 0, BQ16_GPIO_CONF1 + i, 1, BQ_STACK_WRITE);
        // Wait for everyone to get the message
        Align_DelayUs(hbq->htim, 192 + (5 * hbq->NumOfChips));
        if (status != BQ_STATUS_OK)
        {
            return status;
        }
        hbq->GpioConf[i] = data[0]; // Save the config for later
    }

    return status;
}

// Reads ADC and converts them to voltages in place, and puts them on the global bqCellVoltages pointer, whose size should match the max cells variable
// TODO: Convert to a uint8_t return type, to define different error states, which can be handled properly
BQ_StatusTypeDef BQ_GetCellVoltages(BQ_HandleTypeDef *hbq)
{

    BQ_StatusTypeDef status;
    status = BQ_Read(hbq, hbq->BQOutputBuffer, 0, BQ16_VCELL16_HI + (2 * (16 - hbq->NumOfCellsEach)), hbq->NumOfCellsEach * 2, BQ_STACK_READ); // 2 registers for each cell

    if (status != BQ_STATUS_OK)
    {
        return status;
    }

    // If something goes wrong, uncomment this line, as it was removed in the belief that it was wrong (without testing)
    // uint8_t totalLen = 6 + hbq->NumOfCellsEach * hbq->NumOfSlaves * 2; // Totalt expected message length
    uint8_t totalLen = 6 + hbq->NumOfCellsEach * 2; // Totalt expected message length
    // Cleanup
    for (uint8_t i = 0; i < hbq->NumOfSlaves; i++)
    { // Base board will not be part of the cell voltages

        // The responses are always:
        // 1 bytes for message length (minus 1), 1 byte for device id, 2 bytes for register, data inbetween, 2 bytes for CRC

        uint8_t len = hbq->BQOutputBuffer[i * totalLen] + 1; // Should be known, but might as well

        for (uint8_t y = 0; y < len; y += 2)
        {
            uint16_t rawAdc = (((uint16_t)hbq->BQOutputBuffer[i * totalLen + 4 + y]) << 8) | ((uint16_t)hbq->BQOutputBuffer[i * totalLen + 5 + y]);
            hbq->CellVoltages[hbq->NumOfCellsEach * i + y / 2] = (float)((float)rawAdc * 0.00019073); // in mV
        }
    }
    return status;
}

BQ_StatusTypeDef BQ_GetCellTemperatures(BQ_HandleTypeDef *hbq, float beta)
{
    BQ_StatusTypeDef status = BQ_STATUS_OK;

    if (hbq->TempMultiplexEnabled)
    {
        status = BQ_SetGPIOAll(hbq, hbq->TempMultiplexPinIndex, true); // Set GPIO8 to hig

        if (status != BQ_STATUS_OK)
        {
            return status;
        }

        status = BQ_GetGpioMeasurements(hbq, hbq->CellTempPinMap, hbq->BQOutputBuffer); // Read the first set of temperatures

        if (status != BQ_STATUS_OK)
        {

            return status;
        }

        status = BQ_SetGPIOAll(hbq, hbq->TempMultiplexPinIndex, false); // Set GPIO8 to low

        if (status != BQ_STATUS_OK)
        {

            return status;
        }

        status = BQ_GetGpioMeasurements(hbq, hbq->CellTempPinMap, hbq->BQOutputBuffer + (hbq->NumOfTempsEach * hbq->NumOfSlaves * 2)); // 2 Bytes for each temperature
        // Last one is returned either way
    }
    else
    {
        status = BQ_GetGpioMeasurements(hbq, hbq->CellTempPinMap, hbq->BQOutputBuffer);
        // Last one is returned either way
    }

    uint8_t total_num_of_temps = hbq->NumOfTempsEach * hbq->NumOfSlaves; // Total number of temperatures to read
    if(hbq->TempMultiplexEnabled){
        total_num_of_temps *= 2; // If we are multiplexing, we have two sets of temperatures
    }

    for(int i=0; i<total_num_of_temps; i++)
    {
        // Convert the raw ADC values to temperatures in place
        uint16_t adcValue = ((uint16_t *) hbq->BQOutputBuffer)[i];
        float voltage = (((float) adcValue) * 152.59) / 1000.0; // Result in mV

        // Convert the voltage to temperature using the beta formula
        
    }

    return status;
}

BQ_StatusTypeDef BQ_GetDieTemperature(BQ_HandleTypeDef *hbq)
{
    // Cleanup
    BQ_StatusTypeDef status = BQ_Read(hbq, hbq->BQOutputBuffer, 0, BQ16_DIETEMP1_HI, 2, BQ_STACK_READ);

    if (status != BQ_STATUS_OK)
    {
        return status;
    }
    uint8_t totalLen = 8; // pr board
    for (uint8_t i = 0; i < hbq->NumOfSlaves; i++)
    {

        // For now, ignore all CRC checking and verifications, we want the data
        // TODO: Implement proper CRC verification

        uint16_t rawTemp = ((uint16_t)(hbq->BQOutputBuffer[i * totalLen + 4] << 8)) | ((uint16_t)(hbq->BQOutputBuffer[i * totalLen + 4]));
        // As of now, we are only getting the temperature of Die 1
        hbq->BQDieTemperatures[2 * i] = rawTemp * 0.025; // degrees Celcius
    }

    return BQ_Read(hbq, hbq->BQOutputBuffer, 0, BQ16_DIETEMP2_HI, 2, BQ_STACK_READ);
}

// Should be used with bqOutputBuffer
// Output:
// 0 - Everything is ok
// 1 - SPI not ready, and timeout
// 2 - Invalid read type
// 3 - Asking for too much data (max 128 bytes) or too little (under 1)
// 4 - Recieve timeout
BQ_StatusTypeDef BQ_Read(BQ_HandleTypeDef *hbq, uint8_t *pOut, uint8_t deviceId, uint16_t regAddr, uint8_t dataLength, uint8_t readType)
{
    uint32_t start = HAL_GetTick();
    while (BQ_SpiRdy(hbq) != true)
    {
        if (HAL_GetTick() > start + BQ_TIMEOUT)
        {
            return BQ_STATUS_TIMEOUT;
        }
    }
    if ((readType != BQ_DEVICE_READ) && (readType != BQ_STACK_READ) && (readType != BQ_BROAD_READ))
    {
        return BQ_STATUS_DATA_ERROR;
    }

    if (dataLength > 128 || dataLength < 1)
    {
        return BQ_STATUS_DATA_ERROR;
    }

    // Formatting message
    uint8_t writeData[7] = {0};
    uint8_t writeSize = 0;

    writeData[writeSize] = readType;
    writeSize++;
    if (readType == BQ_DEVICE_READ)
    {
        writeData[writeSize] = deviceId;
        writeSize++;
    }
    writeData[writeSize] = (uint8_t)(regAddr >> 8);
    writeSize++;
    writeData[writeSize] = (uint8_t)(regAddr);
    writeSize++;
    writeData[writeSize] = dataLength - 1;
    writeSize++;
    uint16_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t *)writeData, (uint32_t)writeSize);
    // Crc should be sent in reverse
    writeData[writeSize] = (uint8_t)(crc >> 0);
    writeSize++;
    writeData[writeSize] = (uint8_t)(crc >> 8);
    writeSize++;

    // Message should be ready here here

    BQ_SetMosiSPI(hbq); // Getting ready to transmit

    // Transmitting message
    HAL_GPIO_WritePin(hbq->CsPin.GPIOx, hbq->CsPin.Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(hbq->hspi, writeData, writeSize, BQ_TIMEOUT);
    HAL_GPIO_WritePin(hbq->CsPin.GPIOx, hbq->CsPin.Pin, GPIO_PIN_SET);

    // How many bytes do we expect?
    uint16_t maxBytes = 0;

    if (readType == BQ_DEVICE_READ)
    {
        maxBytes = dataLength + 6;
    }
    else if (readType == BQ_STACK_READ)
    {
        maxBytes = (dataLength + 6) * (hbq->NumOfSlaves);
    }
    else if (readType == BQ_BROAD_READ)
    {
        maxBytes = (dataLength + 6) * (hbq->NumOfChips);
    }

    uint16_t fullBuffers = (uint16_t)(maxBytes / 128);
    uint16_t remainingBytes = maxBytes - (fullBuffers * 128);
    BQ_SetMosiIdle(hbq); // We need to transmit 0xFFs.... so why not...?
    while (remainingBytes > 0 || fullBuffers > 0)
    {
        start = HAL_GetTick();
        while (BQ_SpiRdy(hbq) != true)
        {
            if (HAL_GetTick() > start + BQ_TIMEOUT)
            {
                BQ_SetMosiIdle(hbq); // Lets not make the issue bigger than it is
                BQ_ClearComm(hbq);
                return BQ_STATUS_TIMEOUT;
            }
        }

        if (fullBuffers > 0)
        {
            HAL_GPIO_WritePin(hbq->CsPin.GPIOx, hbq->CsPin.Pin, GPIO_PIN_RESET);
            HAL_SPI_Receive(hbq->hspi, pOut, 128, BQ_TIMEOUT);
            HAL_GPIO_WritePin(hbq->CsPin.GPIOx, hbq->CsPin.Pin, GPIO_PIN_SET);
            pOut += 128; // Move pointer 128 steps
            fullBuffers--;
        }
        else
        {
            HAL_GPIO_WritePin(hbq->CsPin.GPIOx, hbq->CsPin.Pin, GPIO_PIN_RESET);
            HAL_SPI_Receive(hbq->hspi, pOut, remainingBytes, BQ_TIMEOUT);
            HAL_GPIO_WritePin(hbq->CsPin.GPIOx, hbq->CsPin.Pin, GPIO_PIN_SET);
            remainingBytes = 0;
        }
    }
    // All data should now be in pOut location (most ofte bqOutputBuffer)

    // CRC Verification
    uint8_t num_of_messages = maxBytes / (dataLength + 6); // Number of messages in the buffer

    for (uint8_t i = 0; i < num_of_messages; i++)
    {

        uint16_t crc_check = HAL_CRC_Calculate(&hcrc, (uint32_t *)(pOut + (i * (dataLength + 6))), (uint32_t)dataLength + 4);

        uint16_t crc_received = ((uint16_t)(pOut[(i * (dataLength + 6)) + dataLength + 5] << 8)) | ((uint16_t)pOut[(i * (dataLength + 6)) + dataLength + 4]);

        if (crc_check != crc_received)
        {
            BQ_SetMosiIdle(hbq); // Lets not make the issue bigger than it is
            return BQ_STATUS_CRC_ERROR;
        }
    }

    BQ_SetMosiIdle(hbq); // Mosi always needs to be idle during end of command

    return BQ_STATUS_OK;
}

// Output:
// 0 - Everything is ok
// 1 - SPI not ready, and timeout
// 2 - Invalid writeType
// 3 - Too much data, over 8 bytes, max supported by device
BQ_StatusTypeDef BQ_Write(BQ_HandleTypeDef *hbq, uint8_t *inData, uint8_t deviceId, uint16_t regAddr, uint8_t dataLength, uint8_t writeType)
{
    uint32_t start = HAL_GetTick();
    while (BQ_SpiRdy(hbq) != true)
    {
        if (HAL_GetTick() > start + BQ_TIMEOUT)
        {
            return BQ_STATUS_TIMEOUT;
        }
    }
    if ((writeType != BQ_DEVICE_WRITE) && (writeType != BQ_STACK_WRITE) && (writeType != BQ_BROAD_WRITE))
    {
        return BQ_STATUS_DATA_ERROR;
    }
    if (dataLength > 8 || dataLength < 1)
    {
        return BQ_STATUS_DATA_ERROR;
    }
    // To limit the program size for now
    uint8_t writeData[14] = {0};

    uint8_t writeSize = 0;
    writeData[writeSize] = writeType | (dataLength - 1);
    writeSize++;

    if (writeType == BQ_DEVICE_WRITE)
    {
        writeData[writeSize] = deviceId;
        writeSize++;
    }
    writeData[writeSize] = (uint8_t)(regAddr >> 8);
    writeSize++;
    writeData[writeSize] = (uint8_t)regAddr;
    writeSize++;

    for (int i = 0; i < dataLength; i++)
    {
        writeData[writeSize] = inData[i];
        writeSize++;
    }

    uint16_t crc = HAL_CRC_Calculate(&hcrc, writeData, writeSize);
    writeData[writeSize] = (uint8_t)(crc >> 0);
    writeSize++;
    writeData[writeSize] = (uint8_t)(crc >> 8);
    writeSize++;

    // Whole transmit should be ready here
    BQ_SetMosiSPI(hbq); // Getting ready to transmit

    HAL_GPIO_WritePin(hbq->CsPin.GPIOx, hbq->CsPin.Pin, GPIO_PIN_RESET);
    Align_DelayUs(hbq->htim, 1); // Atleast 50ns, we dont have that kind of resolution

    HAL_SPI_Transmit(hbq->hspi, writeData, writeSize, BQ_TIMEOUT);

    Align_DelayUs(hbq->htim, 1);
    HAL_GPIO_WritePin(hbq->CsPin.GPIOx, hbq->CsPin.Pin, GPIO_PIN_SET);

    BQ_SetMosiIdle(hbq); // Mosi always needs to be idle during end of command

    return BQ_STATUS_OK;
}