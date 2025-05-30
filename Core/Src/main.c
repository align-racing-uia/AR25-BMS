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
#include "dma.h"
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
#include "secondary_mcu.h"
#include "alignevents.h"
#include "pid.h"
#include "iwdg.h"
#include "faults.h"
#include "ts_statemachine.h"
#include "bms.h"
#include "broadcasts.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Compiliation settings
// #define CONNECTED_TO_BATTERY // For debugging
// #define WATCHDOG_ENABLE      // Enable the watchdog

#define LOW_CURRENT_SENSOR_LIMIT 100 // Amps

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

extern float sim_ocv_map[101];

uint8_t usb_rx_buffer[64] = {0}; // Buffer to store the received USB data
uint8_t usb_rx_len = 0;          // Length of the received data
bool usb_rx_ready = false;       // Flag to indicate that USB data is ready to be processed

uint32_t adc1_buffer[1];
uint32_t adc2_buffer[1];
float low_current_sensor;
float high_current_sensor;

uint32_t pwm_ch3_memory = 0;
uint32_t pwm_ch4_memory = 0;

SecondaryMCU_ResponseTypeDef secondary_response[2] = {0}; // The response from the secondary MCU
SecondaryMCU_TransmitTypeDef secondary_transmit = {0};    // The data to send to the secondary MCU

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

void USB_CDC_RxHandler(uint8_t *Buf, uint32_t Len)
{
  CDC_Transmit_FS(Buf, Len);
}



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
  MX_DMA_Init();
  MX_QUADSPI1_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_CRC_Init();
  MX_USB_Device_Init();
  MX_ADC2_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */


#if defined(WATCHDOG_ENABLE)

  // Enable the watchdog timer
  MX_IWDG_Init();

#endif

  // Initiializ Hardware Peripherals

  // Initialize timer used for PWM generation, and start the DMA
  HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_3, &pwm_ch3_memory, 1); // Start the timer for PWM generation
  HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_4, &pwm_ch4_memory, 1); // Start the timer for PWM generation

  // Start the ADCs in DMA mode
  HAL_ADC_Start_DMA(&hadc1, adc1_buffer, 1);
  HAL_ADC_Start_DMA(&hadc2, adc2_buffer, 1);

  // Set the nFault pin high to indicate no errors on the BMS yet
  HAL_GPIO_WritePin(nFault_GPIO_Port, nFault_Pin, GPIO_PIN_SET); // Set the fault pin high to indicate no fault

  // Initialize timer for align delay
  Align_InitDelay(&htim3); // Initialize the delay function

  // Initialize the CAN interface
  Align_CAN_Init(&hfdcan1, ALIGN_CAN_SPEED_500KBPS, FDCAN1);


  // // Initialize w25q32
  // W25Q_STATE res = W25Q_Init();
  // if (res != W25Q_OK)
  // {
  //   SET_BIT(hbms.ActiveFaults, BMS_NOTE_EEPROM); // Set the EEPROM warning flag
  // }

  // Initilalize the BMS Config
  // BMS_Config_HandleTypeDef bms_config;
  // BMS_Config_Init(&bms_config); // Set the default values of the config

  // // If the W25Q_Init caught a fault, we will not be able to read the config from flash
  // if (!READ_BIT(hbms.ActiveFaults, BMS_NOTE_EEPROM))
  // {
  //   if (BMS_Config_UpdateFromFlash(&bms_config) != BMS_CONFIG_OK)
  //   {
  //     // If we could not read the config from flash, we will set the default values of the config
  //     BMS_Config_Init(&bms_config);                                        // Set the default values of the config (again) as the flash read failed
  //     BMS_Config_StatusTypeDef res = BMS_Config_WriteToFlash(&bms_config); // Write the default values to flash
  //     if (res != BMS_CONFIG_OK)
  //     {
  //       SET_BIT(hbms.ActiveFaults, BMS_NOTE_EEPROM); // Set the EEPROM warning flag
  //       // We have to reset the config to default values, as we could not write to flash and the write reads to verify the write
  //       BMS_Config_Init(&bms_config); // Set the default values of the config
  //     }
  //   }
  // }

  // Currently the BQ handle is defined manually, as there are many parameters that need to be set
  // hbq.hspi = &hspi2;
  // hbq.csGPIOx = GPIOB;
  // hbq.csPin = GPIO_PIN_12;
  // hbq.mosiGPIOx = GPIOB;
  // hbq.mosiPin = GPIO_PIN_15;
  // hbq.spiRdyGPIOx = GPIOB;
  // hbq.spiRdyPin = GPIO_PIN_11;
  // hbq.nFaultGPIOx = GPIOA;
  // hbq.nFaultPin = GPIO_PIN_8;
  // hbq.GpioADCMap = 0x7F;            // All GPIOs are ADCs, except GPIO8, which is an output
  // hbq.CellTempAuxPinMap = 0x7E;   // Pin 1 is used to detect PCB temperature, and the rest are used for the temperature sensors
  // hbq.tempMultiplexEnabled = false; // The temperature sensors are not multiplexed
  // hbq.tempMultiplexPinIndex = 7;    // The pin used to multiplex the temperature sensors, currently 0b01111110
  // hbq.htim = &htim3;                // The timer used for the delays
  // hbq.connected = false;            // The BQ is not connected yet
  
  BQ_HandleTypeDef hbq;
  BQ_PinTypeDef bq_cs_pin = {GPIOB, GPIO_PIN_12}; // Chip select pin for the BQ79600
  BQ_PinTypeDef bq_spi_rdy_pin = {GPIOB, GPIO_PIN_11}; // SPI ready pin for the BQ79600
  BQ_PinTypeDef bq_mosi_pin = {GPIOB, GPIO_PIN_15}; // MOSI pin for the BQ79600
  BQ_PinTypeDef bq_fault_pin = {GPIOA, GPIO_PIN_8}; // Fault pin for the BQ79600

  // Bind Memory regions
  BQ_BindMemory(&hbq, bq_output_buffer, bq_cell_voltages, bq_cell_temperature_pool, bq_die_temperature_pool); // Bind memory pools for the BQ79600 cell voltages
  BQ_BindHardware(&hbq, &hspi2, bq_cs_pin, bq_spi_rdy_pin, bq_mosi_pin, bq_fault_pin, &htim3); // Bind the hardware peripherals to the BQ79600 handle


  // Initialize the Battery Model
  BatteryModel_HandleTypeDef hbm;
  BatteryModel_BindMemory(&hbm, cell_model_memory_pool); // Bind the memory pool to the battery model

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  uint32_t alive_sig_timestamp = HAL_GetTick();
  uint32_t internal_comm_timestamp = HAL_GetTick(); // Communication with the secondary MCU to activate relays

  uint32_t cycle_time_start = 0;
  uint32_t avg_cycle_time = 0;

  PID_HandleTypeDef pid_controller = PID_Init(0.1, 0.01, 0.01, 0.1, 100); // Initialize the PID controller


  BMS_HandleTypeDef hbms;
  BMS_BindMemory(&hbms, &hbm, &hbq); // Initialize the TS state machine


  while (1)
  {
    cycle_time_start = HAL_GetTick(); // Start the cycle time measurement

    // // We should recieve a CAN message atleast as often as the broadcast interval
    // if (can_timeout + bms_config.CanBroadcastInterval < HAL_GetTick())
    // {
    //   // We have not received a CAN message for a while, set the state to fault
    //   SET_BIT(hbms.ActiveFaults, BMS_WARNING_CAN); // Set the CAN warning flag
    // }
    // else
    // {
    //   CLEAR_BIT(hbms.ActiveFaults, BMS_WARNING_CAN); // Clear the CAN warning flag
    // }

    // if (abs(secondary_response[secondary_mcu_recieve_index].PingPongDeviation) > 100) // Check if internal communication is getting slow
    // {
    //   // We have a ping-pong deviation, set the state to fault
    //   SET_BIT(hbms.ActiveFaults, BMS_WARNING_INT_COMM); // Set the CAN warning flag
    // }
    // else
    // {
    //   CLEAR_BIT(hbms.ActiveFaults, BMS_WARNING_INT_COMM); // Clear the CAN warning flag
    // }

    // // Handle data from the secondary MCU

    // sdc_voltage_raw = secondary_response[secondary_mcu_recieve_index].SDCVoltageRaw; // Get the SDC voltage from the secondary MCU
    // hbms.SDC = (((float)sdc_voltage_raw) * 3000.0 / 4096.0) > 2500;                       // Convert to mV

    // Handle the CAN messages
    // if (Align_CAN_Receive(&hfdcan1, &rxHeader, rxData))
    // {
    //   // Recieved a CAN message, reset the timeout
    //   can_timeout = HAL_GetTick();

    //   // Process the received data
    //   uint32_t can_id = rxHeader.Identifier;
    //   uint16_t packet_id = 0;
    //   uint16_t node_id = 0;
    //   Align_SplitCanId(can_id, &packet_id, &node_id, rxHeader.IdType == FDCAN_EXTENDED_ID);

    //   switch (can_id)
    //   {
    //   // We first check for special IDs in case there are devices on the network which do not follow the DTI standard
    //   case 0x18FF50E5: // Insert Charger ID here
    //     /* code */
    //     charger_connected = true;
    //     charger_timeout = HAL_GetTick();
    //     break;

    //   default:
    //     // Using if statements to be able to check against node ids against set variables
    //     if (node_id == bms_config.CanNodeID)
    //     {
    //       BMS_Config_HandleCanMessage(&bms_config, packet_id, rxData); // Handle the CAN message
    //     }
    //     else if (node_id == 1)
    //     { // continoue downwards here
    //       switch (packet_id)
    //       {
    //       case 0x1:
    //         break;
    //       case 0x2:
    //         break;
    //       }
    //     }
    //     else if (node_id == 7) // Simulations and configuration
    //     {
    //       switch (packet_id)
    //       {
    //       }
    //     }
    //   }
    // }

    // Update all the BMS states
    BMS_Update(&hbms);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // Handle the PWM generation
    if (hbq.HighestCellTemperature < 40.0)
    {
      pid_controller.Setpoint = hbq.HighestCellTemperature; // Set the setpoint to the highest cell temperature
    }
    else
    {
      pid_controller.Setpoint = 40.0; // Set the setpoint to 40.0 degrees for now
    }
    float pid_output = fabsf(PID_Compute(&pid_controller, hbq.HighestCellTemperature)); // Calculate the PID output
    if (pid_output > 100.0)
    {
      pid_output = 100.0; // Limit the output to 100%
    }
    else if (pid_output < 0.0)
    {
      pid_output = 0.0; // Limit the output to 0%
    }
    pwm_ch3_memory = (uint32_t)(pid_output / 100.0 * htim2.Instance->ARR); // Set the PWM duty cycle to the PID output, scaled to the PWM resolution
    pwm_ch4_memory = (uint32_t)(pid_output / 100.0 * htim2.Instance->ARR); // Set the PWM duty cycle to the PID output, scaled to the PWM resolution


    if ((internal_comm_timestamp + 50) <= HAL_GetTick())
    {
      HAL_SPI_Transmit(&hspi1, (uint8_t *)&secondary_transmit, sizeof(SecondaryMCU_TransmitTypeDef), 10); // Send the PWM data to the secondary MCU
      HAL_StatusTypeDef status = HAL_SPI_Receive(&hspi1,                                                  // !secondary_mcu_recieve_index is used to place it in the next buffer
                                                 (uint8_t *)&(secondary_response[!secondary_mcu_recieve_index]),
                                                 sizeof(SecondaryMCU_ResponseTypeDef), 10); // Receive the response from the secondary MCU
      if (status != HAL_OK)
      {
        // We have a timeout, set the state to fault
        SET_BIT(hbms.ActiveFaults, BMS_WARNING_INT_COMM); // Set the int comm warning flag
      }
      else
      {
        CLEAR_BIT(hbms.ActiveFaults, BMS_WARNING_INT_COMM); // Clear the int comm warning flag
      }
      internal_comm_timestamp = HAL_GetTick(); // Reset the timer
    }

    // Alive sig ping-pong with the secondary MCU
    if ((alive_sig_timestamp + 250) <= HAL_GetTick())
    {
      // Every second
      HAL_GPIO_TogglePin(GPIOA, Alive_Sig_Pin); // Toggle the alive signal pin
      alive_sig_timestamp = HAL_GetTick();
    }
    // Cycle time filter, pretty agressive, as we want it to spike up when the system is under load
    avg_cycle_time = 0.6 * avg_cycle_time + 0.4 * (HAL_GetTick() - cycle_time_start); // Calculate the cycle time

#if defined(WATCHDOG_ENABLE)
                                                                                      // Refresh the watchdog timer
    HAL_IWDG_Refresh(&hiwdg); // Refresh the watchdog timer

#endif
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSI48 | RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
