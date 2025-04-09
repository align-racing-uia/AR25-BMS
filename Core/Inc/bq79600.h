#ifndef BQ79600_H
#define BQ79600_H
#include "main.h"
#include "bms_config.h"
#include "stdbool.h"

// CONFIG

#define ADC_RES 16

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

#define BQ_CONTROL1_SEND_WAKE 1 << 5
#define BQ_CONTROL1_AA 1 << 0

// Registers for BQ79616
#define BQ16_ACTIVE_CELLS 0x0003
#define BQ16_ADC_CTRL1 0x030D
#define BQ16_ADC_CTRL2 0x030E
#define BQ16_ADC_CTRL3 0x030F

#define BQ16_VCELL16_HI 0x0568  // Everything else is a increment of this + Apply offset for cells less than 16
#define BQ16_GPIO1_HI 0x058E    // The rest is increments of this
#define BQ16_DIETEMP1_HI 0x05AE // Low is a increment of this
#define BQ16_DIETEMP2_HI 0x05B0 // Low is a increment of this

#define BQ16_GPIO_CONF1 0x000E // GPIO_CONF2-4 are increments of this

// Register Flags for BQ79616
#define BQ16_ADC_CTRL1_MAINGO 0x04
#define BQ16_ADC_CTRL1_ADCCONT 0x02
#define BQ16_ADC_CTRL3_AUXGO 0x04
#define BQ16_ADC_CTRL3_AUXCONT 0x02
#define BQ16_GPIO_CONF1_GPIO1_ADC (1 << 1) // These can be used in the relevant positions for the rest as well
#define BQ16_GPIO_CONF1_GPIO2_ADC (1 << 4)
#define BQ16_GPIO_CONF1_GPIO1_OUTPUT 0x5 // These can be used in the relevant positions for the rest as well
#define BQ16_GPIO_CONF1_GPIO2_OUTPUT 0x28 // Defaults to setting the output to low

typedef struct
{
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *csGPIOx;
    GPIO_TypeDef *spiRdyGPIOx;
    GPIO_TypeDef *mosiGPIOx;
    GPIO_TypeDef *nFaultGPIOx;
    uint16_t csPin;
    uint16_t spiRdyPin;
    uint16_t mosiPin;
    uint16_t nFaultPin;
    // This parameter is a bitwise select of what GPIOs should be activated as ADCs
    // And what GPIOs should be treated as General Purpose Outputs
    //    GPIO8 GPIO7 GPIO6 ...
    //  0b  0     0     0   ...
    uint8_t gpioADC;
    uint8_t gpioconf[4]; // This is to skip the need to read the GPIO config registers when setting the GPIOs

    float *cellVoltages;
    float *cellTemperatures;
    // Each board except the master has 2 internal temperature sensors, one on each chip
    float *bqDieTemperatures;
    uint8_t *bqOutputBuffer;
    uint8_t numOfChips;
    uint8_t numOfSlaves;
    uint8_t numOfCellsEach;
    uint8_t numOfTempsEach;
} BQ_HandleTypeDef;

typedef enum
{
    BQ_STATUS_OK = 0,
    BQ_STATUS_SPI_ERROR = 1,
    BQ_STATUS_DATA_ERROR = 2,
    BQ_STATUS_TIMEOUT = 3
} BQ_StatusTypeDef;



void BQ_Init(BQ_HandleTypeDef *hbq);
void BQ_WakePing(BQ_HandleTypeDef *hbq);
void BQ_ClearComm(BQ_HandleTypeDef *hbq);
bool BQ_SpiRdy(BQ_HandleTypeDef *hbq);
void BQ_BindMemory(BQ_HandleTypeDef* hbq, uint8_t num_of_slave_chips, uint8_t *bq_output_buffer, float *cell_voltages_memory_pool, uint8_t num_of_cells_each, float *cell_temperature_memory_pool, uint8_t num_of_temps_each, float *bq_die_temperature_memory_pool);
BQ_StatusTypeDef BQ_SetGPIOAll(BQ_HandleTypeDef *hbq, uint8_t pin, bool logicState);
BQ_StatusTypeDef BQ_WakeMsg(BQ_HandleTypeDef *hbq);
BQ_StatusTypeDef BQ_ActivateSlaveADC(BQ_HandleTypeDef *hbq);
BQ_StatusTypeDef BQ_ActivateAuxADC(BQ_HandleTypeDef* hbq);
BQ_StatusTypeDef BQ_ConfigureGPIO(BQ_HandleTypeDef *hbq);
BQ_StatusTypeDef BQ_GetCellVoltages(BQ_HandleTypeDef *hbq);
BQ_StatusTypeDef BQ_GetCellTemperatures(BQ_HandleTypeDef *hbq);
BQ_StatusTypeDef BQ_AutoAddress(BQ_HandleTypeDef *hbq);
BQ_StatusTypeDef BQ_GetDieTemperatures(BQ_HandleTypeDef *hbq);

BQ_StatusTypeDef BQ_Read(BQ_HandleTypeDef *hbq, uint8_t *pOut, uint8_t deviceId, uint16_t regAddr, uint8_t dataLength, uint8_t readType);
BQ_StatusTypeDef BQ_Write(BQ_HandleTypeDef *hbq, uint8_t *inData, uint8_t deviceId, uint16_t regAddr, uint8_t dataLength, uint8_t writeType);

#endif