#ifndef BQ79600_H
#define BQ79600_H
#include "main.h"
#include "stdbool.h"
#include "bms_config.h"

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
#define BQ16_ADC_CONF1 0x0007 // ADC_CONF2-4 are increments of this

// Register Flags for BQ79616
#define BQ16_ADC_CTRL1_MAINGO 0x04
#define BQ16_ADC_CTRL1_ADCCONT 0x02
#define BQ16_ADC_CTRL3_AUXGO 0x04
#define BQ16_ADC_CTRL3_AUXCONT 0x02
#define BQ16_GPIO_CONF1_GPIO1_ADC (1 << 1) // These can be used in the relevant positions for the rest as well
#define BQ16_GPIO_CONF1_GPIO2_ADC (1 << 4)
#define BQ16_GPIO_CONF1_GPIO1_OUTPUT 0x5  // These can be used in the relevant positions for the rest as well
#define BQ16_GPIO_CONF1_GPIO2_OUTPUT 0x28 // Defaults to setting the output to low

#define BQ16_ADC_CONF1_LPF_13HZ 0x1
#define BQ16_ADC_CONF1_LPF_26HZ 0x2
#define BQ16_ADC_CONF1_LPF_53HZ 0x3

#define BQ16_COMM_TIMEOUT_CONF 0x0019
#define BQ16_CONTROL2 0x030A
#define BQ16_TSREF_HI 0x058C // TSREF pin control register high byte

typedef struct {
    GPIO_TypeDef *GPIOx; // The GPIO port
    uint16_t Pin;        // The GPIO pin
} BQ_PinTypeDef;

typedef struct {
    uint8_t NumOfSlaves; // Number of slave chips
    uint8_t NumOfCellsEach; // Number of cells each chip should measure
    uint8_t NumOfTempsEach; // Number of temperature sensors each chip should measure

    bool TempMultiplexEnabled;     // If the temperature sensors are multiplexed
    uint8_t TempMultiplexPinIndex; // The pin used to multiplex the temperature sensors
    uint8_t GpioAuxADCMap;
    uint8_t FirstTempGPIO;        // The pin map for the cell temperature sensors

} BQ_ConfigTypeDef;

typedef struct
{
    SPI_HandleTypeDef *hspi;
    BQ_PinTypeDef CsPin;
    BQ_PinTypeDef SpiRdyPin;
    BQ_PinTypeDef MosiPin;
    BQ_PinTypeDef FaultPin;
    // This parameter is a bitwise select of what GPIOs should be activated as ADCs
    // And what GPIOs should be treated as General Purpose Outputs
    //    GPIO8 GPIO7 GPIO6 ...
    //  0b  0     0     0   ...
    uint8_t GpioAuxADCMap;
    uint8_t GpioConf[4]; // This is to skip the need to read the GPIO config registers when setting the GPIOs

    float HighestCellTemperature;
    float LowestCellTemperature;
    float *CellVoltages;
    float *CellTemperatures;
    uint8_t *RawCellTemperatures; // This is the raw cell temperatures read from the BQ79600
    // Each board except the master has 2 internal temperature sensors, one on each chip
    float *BQDieTemperatures;
    uint8_t *BQOutputBuffer;
    uint8_t NumOfChips;
    uint8_t NumOfSlaves;
    uint8_t NumOfCellsEach;
    uint8_t NumOfTempsEach;

    bool TempMultiplexEnabled;     // This is true if the temperature sensors are multiplexed
    uint8_t TempMultiplexPinIndex; // This is the pin used to multiplex the temperature sensors
    uint8_t FirstTempGPIO;

    bool MultiplexToggle;


    TIM_HandleTypeDef *htim; // The timer used for the delays
} BQ_HandleTypeDef;

typedef enum
{
    BQ_STATUS_OK = 0,
    BQ_STATUS_SPI_ERROR = 1,
    BQ_STATUS_DATA_ERROR = 2,
    BQ_STATUS_TIMEOUT = 3,
    BQ_STATUS_CRC_ERROR = 4,
} BQ_StatusTypeDef;

void BQ_Init(BQ_HandleTypeDef *hbq);
void BQ_Configure(BQ_HandleTypeDef *hbq, BQ_ConfigTypeDef *bq_config);
void BQ_BindMemory(BQ_HandleTypeDef *hbq, uint8_t *bq_output_buffer, float *cell_voltages_memory_pool, uint8_t *raw_cell_temperature_memory_pool, float *cell_temperature_memory_pool, float *bq_die_temperature_memory_pool);
void BQ_BindHardware(BQ_HandleTypeDef *hbq, SPI_HandleTypeDef *hspi, BQ_PinTypeDef cs_pin, BQ_PinTypeDef spi_rdy_pin, BQ_PinTypeDef mosi_pin, BQ_PinTypeDef fault_pin, TIM_HandleTypeDef *htim);
void BQ_WakePing(BQ_HandleTypeDef *hbq);
void BQ_ClearComm(BQ_HandleTypeDef *hbq);
bool BQ_SpiRdy(BQ_HandleTypeDef *hbq);
BQ_StatusTypeDef BQ_SetGPIOAll(BQ_HandleTypeDef *hbq, uint8_t pin, bool logicState);
BQ_StatusTypeDef BQ_WakeMsg(BQ_HandleTypeDef *hbq);
BQ_StatusTypeDef BQ_ConfigureMainADC(BQ_HandleTypeDef *hbq);
BQ_StatusTypeDef BQ_ActivateMainADC(BQ_HandleTypeDef *hbq);
BQ_StatusTypeDef BQ_ActivateAuxADC(BQ_HandleTypeDef *hbq);
BQ_StatusTypeDef BQ_ConfigureGPIO(BQ_HandleTypeDef *hbq);
BQ_StatusTypeDef BQ_GetCellVoltages(BQ_HandleTypeDef *hbq);
BQ_StatusTypeDef BQ_GetCellTemperatures(BQ_HandleTypeDef *hbq, float beta);
BQ_StatusTypeDef BQ_GetGpioMeasurements(BQ_HandleTypeDef *hbq, uint8_t pin_map, uint8_t *data_out);
BQ_StatusTypeDef BQ_EnableTsRef(BQ_HandleTypeDef *hbq);
BQ_StatusTypeDef BQ_EnableCommTimeout(BQ_HandleTypeDef *hbq);
BQ_StatusTypeDef BQ_AutoAddress(BQ_HandleTypeDef *hbq);
BQ_StatusTypeDef BQ_GetDieTemperatures(BQ_HandleTypeDef *hbq);

BQ_StatusTypeDef BQ_Read(BQ_HandleTypeDef *hbq, uint8_t *pOut, uint8_t deviceId, uint16_t regAddr, uint8_t dataLength, uint8_t readType);
BQ_StatusTypeDef BQ_Write(BQ_HandleTypeDef *hbq, uint8_t *inData, uint8_t deviceId, uint16_t regAddr, uint8_t dataLength, uint8_t writeType);

#endif