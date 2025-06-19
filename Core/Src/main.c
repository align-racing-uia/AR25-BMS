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
#include "quadspi.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "bms_config.h"
#include "stdbool.h"
#include "SEGGER_RTT.h"
#include "alignutils.h"
#include "aligncan.h"
#include "bq79600.h"
#include "w25q_mem.h"
#include "string.h"
#include "battery_model.h"
#include "alignevents.h"
#include "pid.h"
#include "iwdg.h"
#include "faults.h"
#include "bms.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// These are absolute maxes for the battery model, not the actual values
// The actual values are set in the battery model init function

#define TEMP_MAP_POOL_MAX_POINTS 100 // Size of the OCV map pool, defaults to a maximum of 5 temperature maps with 15 points each
#define TEMP_MAP_POOL_AMOUNT 5       // Size of the temperature map pool, defaults to a maximum of 5 temperature maps with 15 points each

#define BQ_MAX_AMOUNT_OF_CHIPS 15                            // The maximum amount of chips in the system
#define BQ_MAX_AMOUNT_OF_SLAVES (BQ_MAX_AMOUNT_OF_CHIPS - 1) // The maximum amount of BQ79616 chips in the system
#define BQ_MAX_AMOUNT_OF_CELLS_EACH 16                       // The maximum amount of cells in series on each board
#define BQ_MAX_AMOUNT_OF_TEMPS_EACH 14                       // The maximum amount of temperature sensors on each board

#define CELL_MEMORY_POOL_SIZE (BQ_MAX_AMOUNT_OF_SLAVES * BQ_MAX_AMOUNT_OF_CELLS_EACH) // Size of the cell memory pool, defaults to a maximum of 300 cells

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
float sim_ocv_map[101] = {
    2907.03,
    3021.21,
    3116.98,
    3185.15,
    3239.26,
    3284.97,
    3323.05,
    3355.40,
    3382.05,
    3402.41,
    3418.22,
    3431.19,
    3441.68,
    3450.80,
    3459.78,
    3468.63,
    3477.72,
    3487.24,
    3497.56,
    3508.97,
    3519.96,
    3530.81,
    3541.47,
    3551.71,
    3561.86,
    3571.15,
    3579.96,
    3588.14,
    3595.65,
    3603.04,
    3610.06,
    3616.73,
    3623.12,
    3629.34,
    3635.58,
    3641.95,
    3648.20,
    3654.77,
    3661.46,
    3668.54,
    3675.52,
    3683.01,
    3690.63,
    3698.58,
    3706.65,
    3715.03,
    3723.80,
    3732.35,
    3741.03,
    3750.03,
    3758.76,
    3767.81,
    3776.57,
    3785.47,
    3794.56,
    3803.86,
    3813.02,
    3822.78,
    3832.66,
    3842.37,
    3851.54,
    3860.47,
    3868.98,
    3877.00,
    3884.63,
    3892.32,
    3899.32,
    3906.63,
    3913.96,
    3921.29,
    3929.20,
    3937.26,
    3946.02,
    3955.38,
    3965.09,
    3975.40,
    3986.15,
    3997.05,
    4007.91,
    4018.71,
    4029.22,
    4038.90,
    4048.19,
    4056.58,
    4063.70,
    4069.65,
    4074.40,
    4078.24,
    4081.59,
    4084.89,
    4088.05,
    4091.90,
    4095.77,
    4100.49,
    4106.10,
    4112.92,
    4121.36,
    4131.89,
    4145.73,
    4163.89,
    4183.32};

uint8_t usb_rx_buffer[64] = {0}; // Buffer to store the received USB data
uint8_t usb_rx_len = 0;          // Length of the received data
bool usb_rx_ready = false;       // Flag to indicate that USB data is ready to be processed

uint16_t adc1_buffer[1] = {0};
uint16_t adc2_buffer[2] = {0};

uint32_t pwm_ch3_memory = 0;
uint32_t pwm_ch4_memory = 0;

// Create memory pools for the battery models
// This is done here to make it transparent to the user
CellModel_HandleTypeDef cell_model_memory_pool[CELL_MEMORY_POOL_SIZE];

// Memory pools for the BQ79600
// This is done here to make it transparent to the user
uint8_t bq_output_buffer[BQ_MAX_AMOUNT_OF_CHIPS * 128];                                          // This is the memory pool for the BQ79600 output buffer
float bq_cell_voltages[BQ_MAX_AMOUNT_OF_SLAVES * BQ_MAX_AMOUNT_OF_CELLS_EACH];                   // This is the memory pool for the cell voltages
float bq_die_temperature_pool[2 * BQ_MAX_AMOUNT_OF_SLAVES];                                      // This is the memory pool for the die temperatures
float bq_cell_temperature_pool[BQ_MAX_AMOUNT_OF_SLAVES * BQ_MAX_AMOUNT_OF_TEMPS_EACH];           // This is the memory pool for the cell temperatures
uint8_t bq_raw_cell_temperature_pool[BQ_MAX_AMOUNT_OF_SLAVES * BQ_MAX_AMOUNT_OF_TEMPS_EACH * 2]; // This is the memory pool for the cell temperatures

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_CRC_Init();
  MX_ADC2_Init();
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

  // Set the nFault pin high to indicate no errors on the BMS yet
  HAL_GPIO_WritePin(nFault_GPIO_Port, nFault_Pin, GPIO_PIN_RESET); // Set the fault pin low to indicate no fault

  // Initialize timer for align delay
  Align_InitDelay(&htim3); // Initialize the delay function

  // Initialize the CAN interface
  Align_CAN_Init(&hfdcan1, ALIGN_CAN_SPEED_500KBPS, FDCAN1);

  BQ_HandleTypeDef hbq;
  BQ_PinTypeDef bq_cs_pin = {GPIOB, GPIO_PIN_12};      // Chip select pin for the BQ79600
  BQ_PinTypeDef bq_spi_rdy_pin = {GPIOB, GPIO_PIN_11}; // SPI ready pin for the BQ79600
  BQ_PinTypeDef bq_mosi_pin = {GPIOB, GPIO_PIN_15};    // MOSI pin for the BQ79600
  BQ_PinTypeDef bq_fault_pin = {GPIOA, GPIO_PIN_8};    // Fault pin for the BQ79600

  // Bind Memory regions
  BQ_BindMemory(&hbq, bq_output_buffer, bq_cell_voltages, bq_raw_cell_temperature_pool, bq_cell_temperature_pool, bq_die_temperature_pool); // Bind memory pools for the BQ79600 cell voltages
  BQ_BindHardware(&hbq, &hspi2, bq_cs_pin, bq_spi_rdy_pin, bq_mosi_pin, bq_fault_pin, &htim3);                                              // Bind the hardware peripherals to the BQ79600 handle

  // Initialize the Battery Model
  BatteryModel_HandleTypeDef hbm;
  BatteryModel_BindMemory(&hbm, cell_model_memory_pool, sim_ocv_map, 101); // Bind the memory pool to the battery model

  BMS_HandleTypeDef hbms;
  BMS_HardwareConfigTypeDef bms_hardware_config = {
      .hfdcan = &hfdcan1,                                                               // Set the FDCAN handle
      .FaultPin = {AMS_Fault_GPIO_Port, AMS_Fault_Pin},                                 // Set the fault pin
      .LowCurrentSensorPin = {Low_Current_Sensor_GPIO_Port, Low_Current_Sensor_Pin},    // Set the low current sensor pin
      .HighCurrentSensorPin = {High_Current_Sensor_GPIO_Port, High_Current_Sensor_Pin}, // Set the high current sensor pin
      .MinusAIR = {Minus_GPIO_Port, Minus_Pin},                                         // Set the minus AIR pin
      .PlusAIR = {Plus_GPIO_Port, Plus_Pin},                                            // Set the plus AIR pin
      .PrechargeAIR = {Precharge_GPIO_Port, Precharge_Pin},                             // Set the precharge AIR pin¨
      .SdcPin = {SDC_GPIO_Port, SDC_Pin}                                                // Set the SDC closed pin
  };

  BMS_BindMemory(&hbms, &hbm, &hbq);     // Initialize the TS state machine
  BMS_Init(&hbms, &bms_hardware_config); // Initialize the BMS state machine

  // PID Controller for fan control - Not used in the BMS, but needed for the ACU functionality
  PID_HandleTypeDef pid_controller = PID_Init(0.1, 0.01, 0.01, 0.1, 100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  uint32_t alive_sig_timestamp = HAL_GetTick();

  uint32_t cycle_time_start = 0;
  uint32_t avg_cycle_time = 0;

  while (1)
  {
    cycle_time_start = HAL_GetTick(); // Start the cycle time measurement

    // Start polling the ADCs in DMA mode
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc1_buffer, 1);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc2_buffer, 2);

    // Update all the BMS states
    BMS_Update(&hbms);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // Functionality not directly related to the BMS, but still needed for the BMS to work as an ACU
    float ntc_voltage = ((float)adc2_buffer[1]) / 4095 * 2900; // Read the voltage (in mV) from the external temperature sensor
    float ntc_resistance = ((3300.0 * 10000.0) / (ntc_voltage)) - 10000.0;
    float ntc_temp = K_TO_C(1.0 / ((1.0 / C_TO_K(25.0f)) + (1 / 4300.0) * logf(ntc_resistance / 10000.0f))); // Calculate the temperature in Celsius from the NTC voltage

    // Handle the PWM generation
    // If the highest cell temperature is above the NTC temperature (ambient), set the PID setpoint to the NTC temperature
    if (hbq.HighestCellTemperature >= ntc_temp)
    {
      pid_controller.Setpoint = ntc_temp; // Set the setpoint to 40.0 degrees for now
    }
    else
    {
      // If the pack somehow is cooler than the ambient temperature, set the PID setpoint to the highest cell temperature
      // This is to prevent the PID from trying to cool the pack when it is below ambient temperature
      pid_controller.Setpoint = hbq.HighestCellTemperature; // Set the setpoint to the highest cell temperature
    }

    float pid_output = fabsf(PID_Compute(&pid_controller, hbq.HighestCellTemperature)); // Calculate the PID output
    if (pid_output > 100.0)
    {
      pid_output = 100.0; // Limit the output to 100%
    }
    else if (pid_output < 0.0) // It makes no sensor to have a negative output, as the fan cannot run in reverse, neither can the PWM signal be negative
    {
      pid_output = 0.0; // Limit the output to 0%
    }
    pwm_ch3_memory = (uint32_t)(pid_output / 100.0 * htim2.Instance->ARR); // Set the PWM duty cycle to the PID output, scaled to the PWM resolution
    pwm_ch4_memory = (uint32_t)(pid_output / 100.0 * htim2.Instance->ARR); // Set the PWM duty cycle to the PID output, scaled to the PWM resolution

    // Alive sig ping-pong with the secondary MCU
    if ((alive_sig_timestamp + 100) <= HAL_GetTick())
    {
      // Feed the external watchdog
      // Note: This cannot be disabled, as it is an external component
      HAL_GPIO_TogglePin(Ext_WD_Sig_GPIO_Port, Ext_WD_Sig_Pin); // Toggle the alive signal pin
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

  /** Configure the main internal regulator output voltage
   */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
