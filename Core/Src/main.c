/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "crc.h"
#include "i2c.h"
#include "quadspi.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "bms_config.h"
#include "usbd_cdc_if.h"
#include "stdbool.h"
#include "SEGGER_RTT.h"
#include "alignutils.h"
#include "aligncan.h"
#include "bq79600.h"
#include "w25q_mem.h"
#include "string.h"
#include "battery_model.h"
#include "icm.h"
#include "usb_logging.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// TODO: Find actual limit
#define LOW_CURRENT_SENSOR_LIMIT 100 // Amps
#define USB_LOGGING 1 // Enable USB logging

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint32_t adc1Buffer[1];
uint32_t adc2Buffer[1];
float lowCurrentSensor;
float highCurrentSensor;

// Create memory pools for the battery models
// This is done here to make it transparent to the user
CellModel_HandleTypeDef cell_model_memory_pool[CELL_MEMORY_POOL_SIZE];
float temp_map_voltage_points[TEMP_MAP_POOL_MAX_POINTS * TEMP_MAP_POOL_AMOUNT];
float temp_map_soc_points[TEMP_MAP_POOL_MAX_POINTS];
TempMap_HandleTypeDef temp_map_pool[TEMP_MAP_POOL_AMOUNT];

// Memory pools for the BQ79600
// This is done here to make it transparent to the user
uint8_t bq_output_buffer[BQ_MAX_AMOUNT_OF_CHIPS * 128];                                // This is the memory pool for the BQ79600 output buffer
float bq_cell_voltages[BQ_MAX_AMOUNT_OF_SLAVES * BQ_MAX_AMOUNT_OF_CELLS_EACH];         // This is the memory pool for the cell voltages
float bq_die_temperature_pool[2 * BQ_MAX_AMOUNT_OF_SLAVES];                            // This is the memory pool for the die temperatures
float bq_cell_temperature_pool[BQ_MAX_AMOUNT_OF_SLAVES * BQ_MAX_AMOUNT_OF_TEMPS_EACH]; // This is the memory pool for the cell temperatures

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void UpdateCurrentSensor(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

  /* USER CODE BEGIN 1 */
  SEGGER_RTT_Init();
  

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_QUADSPI1_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_CRC_Init();
  MX_USB_Device_Init();
  MX_ADC2_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  // Initialize timer for align delay
  HAL_TIM_Base_Start(&htim2);

  // Initialize w25q32
  W25Q_STATE res = W25Q_Init();
  if (res != W25Q_OK)
  {
    Error_Handler(); // Hard stop if this fails
  }

  // Initilalize the BMS Config
  BMS_ConfigTypeDef bms_config;
  bool valid_config = false;

  // Read the config from the flash: The max size is of the config is currently one page (256 bytes)
  if (W25Q_ReadData((uint8_t *)&bms_config, sizeof(BMS_ConfigTypeDef), 0, 0) != W25Q_OK)
  {
    Error_Handler(); // Hard stop if this fails
    // Someone mustve pulled the chip out, or something else went wrong
  }



  // Check if memory contains a config, and that it is a valid
  if (strncmp(bms_config.MemoryCheck, "align", 5) == 0)
  {

    // Verify checksum:
    // TODO Implement a good checksum
    // BMS_Config_UpdateFromFlash(&bms_config, (uint8_t *)&bms_config, sizeof(BMS_ConfigTypeDef));
    valid_config = true;
  }

  if (!valid_config)
  {
    // If the config is not valid, we should set it to default values
    bms_config.ConfigVersion = 1;
    bms_config.BroadcastPacket = DEFAULT_CAN_BROADCAST_PACKET;
    bms_config.CellCount = DEFAULT_TOTAL_CELLS;
    bms_config.NumOfChips = DEFAULT_TOTAL_CHIPS;
    bms_config.NumOfSlaves = DEFAULT_TOTAL_CHIPS - 1; // The master is not counted as a slave
    bms_config.CellsEach = DEFAULT_CELLS_EACH;
    bms_config.TempsEach = DEFAULT_TEMPS_EACH;
    bms_config.TempMapVoltagePoints = DEFAULT_TEMP_MAP_VOLTAGE_POINTS;
    bms_config.TempMapAmount = DEFAULT_TEMP_MAP_AMOUNT;
    bms_config.TotalCellCountInSeries = DEFAULT_TOTAL_CELLS_IN_SERIES;
    bms_config.CellCountInParallel = DEFAULT_CELLS_IN_PARALLEL;
    bms_config.CellVoltageLimitLow = DEFAULT_CELLVOLTAGE_LIMIT_LOW;
    bms_config.CellVoltageLimitHigh = DEFAULT_CELLVOLTAGE_LIMIT_HIGH;
    bms_config.CellTemperatureLimitLow = DEFAULT_CELLTEMPERATURE_LIMIT_LOW;   // -40C
    bms_config.CellTemperatureLimitHigh = DEFAULT_CELLTEMPERATURE_LIMIT_HIGH; // 85C
    bms_config.CanNodeID = DEFAULT_CAN_NODE_ID;
    bms_config.CanBaudrate = DEFAULT_CAN_BAUDRATE;
    bms_config.Checksum = 0x00; // TODO: Implement a good checksum
  }

  BQ_HandleTypeDef hbq;
  hbq.hspi = &hspi2;
  hbq.csGPIOx = GPIOB;
  hbq.csPin = GPIO_PIN_12;
  hbq.mosiGPIOx = GPIOB;
  hbq.mosiPin = GPIO_PIN_15;
  hbq.spiRdyGPIOx = GPIOB;
  hbq.spiRdyPin = GPIO_PIN_11;
  hbq.nFaultGPIOx = GPIOA;
  hbq.nFaultPin = GPIO_PIN_8;
  hbq.gpioADC = 0x7F; // All GPIOs are ADCs, except GPIO8, which is an output

  BQ_BindMemory(&hbq, bms_config.NumOfSlaves, bq_output_buffer, bq_cell_voltages, bms_config.CellsEach, bq_cell_temperature_pool, bms_config.TempsEach, bq_die_temperature_pool); // Bind memory pools for the BQ79600 cell voltages

  // Init BQ79600 (Could be wrapped into one function)
  BQ_StatusTypeDef status;
  BQ_WakePing(&hbq);
  BQ_WakePing(&hbq);

  status = BQ_WakeMsg(&hbq);
  if (status != BQ_STATUS_OK)
  {
    Error_Handler(); // Hard stop if this fails
  }
  BQ_ClearComm(&hbq);
  status = BQ_AutoAddress(&hbq);
  if (status != BQ_STATUS_OK)
  {
    Error_Handler(); // Hard stop if this fails
  }
  status = BQ_ConfigureGPIO(&hbq);
  if (status != BQ_STATUS_OK)
  {
    Error_Handler(); // Hard stop if this fails
  }

  BQ_ActivateSlaveADC(&hbq); // Activate the ADC on all slaves
  if (status != BQ_STATUS_OK)
  {
    Error_Handler(); // Hard stop if this fails
  }

  HAL_ADC_Start_DMA(&hadc1, adc1Buffer, 1);
  HAL_ADC_Start_DMA(&hadc2, adc2Buffer, 1);

  BatteryModel_HandleTypeDef battery_model;
  BatteryModel_Init(&battery_model, cell_model_memory_pool, bms_config.CellCount, bms_config.TotalCellCountInSeries, bms_config.CellCountInParallel);
  BatteryModel_InitOCVMaps(&battery_model, bms_config.TempMapVoltagePoints, temp_map_voltage_points, temp_map_soc_points, temp_map_pool, bms_config.TempMapAmount);
  BatteryModel_LoadCellData(&battery_model, 0, 0, 0, 0 ,0, 0, 0, 0); // Load the cell data into the battery model

  Align_CAN_Init(&hfdcan1, ALIGN_CAN_SPEED_500KBPS, FDCAN1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  bool toggle = false;
  bool charger_connected = false;

  FDCAN_RxHeaderTypeDef rxHeader;
  uint8_t rxData[8];

  uint32_t can_timestamp = HAL_GetTick();
  uint32_t usb_timestamp = HAL_GetTick();
  uint32_t charger_timestamp = HAL_GetTick();
  uint32_t charger_timeout = HAL_GetTick();
  USB_LogFrameTypeDef usb_log = {0};


  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // The main tasks of the BMS
    BQ_GetCellVoltages(&hbq);

    BQ_GetCellTemperatures(&hbq);

    // UpdateCurrentSensor();

    float currentSensor = lowCurrentSensor;
    // We rely on the low current sensor to be the most accurate, and the high current sensor when we are in the high current region
    if (lowCurrentSensor >= LOW_CURRENT_SENSOR_LIMIT || lowCurrentSensor <= LOW_CURRENT_SENSOR_LIMIT)
    {
      currentSensor = highCurrentSensor;
    }

    BatteryModel_UpdateMeasured(&battery_model, hbq.cellVoltages, hbq.cellTemperatures, &currentSensor);
    BatteryModel_UpdateEstimates(&battery_model);

    // We do communication at the end
    if (Align_CAN_Receive(&hfdcan1, &rxHeader, rxData))
    {
      // Process the received data
      uint32_t can_id = rxHeader.Identifier;
      uint8_t packet_id = 0;
      uint8_t node_id = 0;
      Align_SplitCanId(can_id, &packet_id, &node_id, rxHeader.IdType == FDCAN_EXTENDED_ID);

      switch (can_id)
      {
      // We first check for special IDs in case there are devices on the network which do not follow the DTI standard
      case 0x18FF50E5: // Insert Charger ID here
        /* code */
        charger_connected = true;
        charger_timeout = HAL_GetTick();
        break;

      default:
        switch (node_id)
        {

        case 12:
          if(packet_id == 0x1F){
            // BMS_Config_WriteToFlash(&bms_config, );

          }
          BMS_Config_HandleCanMessage(&bms_config, packet_id, rxData);
        default:
          break;
        }
      }
    }

    if ((can_timestamp + 1000) <= HAL_GetTick())
    {
      // Every second
      uint8_t data[8] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
      uint32_t can_id = Align_CombineCanId(bms_config.CanNodeID, bms_config.BroadcastPacket, bms_config.CanExtended);
      Align_CAN_Send(&hfdcan1, can_id, data, 8, bms_config.CanExtended);
      can_timestamp = HAL_GetTick();
      toggle = !toggle;
      BQ_SetGPIOAll(&hbq, 7, toggle);      // Set GPIO8 to high
    }
    
    if((usb_timestamp + 10) <= HAL_GetTick() && USB_LOGGING){
      // Every second
      CDC_Transmit_FS((uint8_t*) &usb_log, sizeof(usb_log)); // Send data to USB CDC
      usb_timestamp = HAL_GetTick();
    }

    if(charger_connected && ((charger_timestamp + 1000) <= HAL_GetTick())){
      // Every second

      uint16_t max_charing_voltage = 5880; // 588.0 V
      uint16_t max_charing_current = 100;  // 10.0 A
      uint8_t  charger_enabed = 1; // 1 = disabled, 0 = enabled
      uint8_t charger_data[8] = {0};
      charger_data[0] = (max_charing_voltage >> 8) & 0xFF; // High byte
      charger_data[1] = max_charing_voltage & 0xFF;        // Low byte
      charger_data[2] = (max_charing_current >> 8) & 0xFF; // High byte
      charger_data[3] = max_charing_current & 0xFF;        // Low byte
      charger_data[4] = charger_enabed;                   // Enable charger

      Align_CAN_Send(&hfdcan1, 0x1806E5F4, charger_data, 8, true); // Send data to the charger

      if(charger_timeout + 5000 <= HAL_GetTick()){
        // Charger timeout
        charger_connected = false;
        charger_timeout = HAL_GetTick();
      }
      charger_timestamp = HAL_GetTick();
    }
  }

  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_CRSInitTypeDef pInit = {0};

  /** Configure the main internal regulator output voltage
   */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSI48 | RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV3;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the SYSCFG APB clock
   */
  __HAL_RCC_CRS_CLK_ENABLE();

  /** Configures CRS
   */
  pInit.Prescaler = RCC_CRS_SYNC_DIV1;
  pInit.Source = RCC_CRS_SYNC_SOURCE_USB;
  pInit.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  pInit.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000, 1000);
  pInit.ErrorLimitValue = 34;
  pInit.HSI48CalibrationValue = 32;

  HAL_RCCEx_CRSConfig(&pInit);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM1 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
