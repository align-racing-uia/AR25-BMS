/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    quadspi.c
  * @brief   This file provides code for the configuration
  *          of the QUADSPI instances.
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
#include "quadspi.h"

/* USER CODE BEGIN 0 */
static uint8_t QSPI_WriteEnable(void);
uint8_t QSPI_AutoPollingMemReady(void);
static uint8_t QSPI_Configuration(void);
static uint8_t QSPI_ResetChip(void);

/* USER CODE END 0 */

QSPI_HandleTypeDef hqspi1;

/* QUADSPI1 init function */
void MX_QUADSPI1_Init(void)
{

  /* USER CODE BEGIN QUADSPI1_Init 0 */

  /* USER CODE END QUADSPI1_Init 0 */

  /* USER CODE BEGIN QUADSPI1_Init 1 */

  /* USER CODE END QUADSPI1_Init 1 */
  hqspi1.Instance = QUADSPI;
  hqspi1.Init.ClockPrescaler = 0;
  hqspi1.Init.FifoThreshold = 4;
  hqspi1.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
  hqspi1.Init.FlashSize = 21;
  hqspi1.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi1.Init.ClockMode = QSPI_CLOCK_MODE_0;
  hqspi1.Init.FlashID = QSPI_FLASH_ID_1;
  hqspi1.Init.DualFlash = QSPI_DUALFLASH_DISABLE;
  if (HAL_QSPI_Init(&hqspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI1_Init 2 */

  /* USER CODE END QUADSPI1_Init 2 */

}

void HAL_QSPI_MspInit(QSPI_HandleTypeDef* qspiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(qspiHandle->Instance==QUADSPI)
  {
  /* USER CODE BEGIN QUADSPI_MspInit 0 */

  /* USER CODE END QUADSPI_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_QSPI;
    PeriphClkInit.QspiClockSelection = RCC_QSPICLKSOURCE_HSI;

    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* QUADSPI clock enable */
    __HAL_RCC_QSPI_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**QUADSPI1 GPIO Configuration
    PA2     ------> QUADSPI1_BK1_NCS
    PA3     ------> QUADSPI1_CLK
    PA6     ------> QUADSPI1_BK1_IO3
    PA7     ------> QUADSPI1_BK1_IO2
    PB0     ------> QUADSPI1_BK1_IO1
    PB1     ------> QUADSPI1_BK1_IO0
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF10_QUADSPI;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF10_QUADSPI;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN QUADSPI_MspInit 1 */

  /* USER CODE END QUADSPI_MspInit 1 */
  }
}

void HAL_QSPI_MspDeInit(QSPI_HandleTypeDef* qspiHandle)
{

  if(qspiHandle->Instance==QUADSPI)
  {
  /* USER CODE BEGIN QUADSPI_MspDeInit 0 */

  /* USER CODE END QUADSPI_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_QSPI_CLK_DISABLE();

    /**QUADSPI1 GPIO Configuration
    PA2     ------> QUADSPI1_BK1_NCS
    PA3     ------> QUADSPI1_CLK
    PA6     ------> QUADSPI1_BK1_IO3
    PA7     ------> QUADSPI1_BK1_IO2
    PB0     ------> QUADSPI1_BK1_IO1
    PB1     ------> QUADSPI1_BK1_IO0
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_6|GPIO_PIN_7);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_0|GPIO_PIN_1);

  /* USER CODE BEGIN QUADSPI_MspDeInit 1 */

  /* USER CODE END QUADSPI_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/* QUADSPI init function */
uint8_t CSP_QUADSPI_Init(void) {

	//prepare QSPI peripheral for ST-Link Utility operations
	hqspi1.Instance = QUADSPI;
	if (HAL_QSPI_DeInit(&hqspi1) != HAL_OK) {
		return HAL_ERROR;
	}

	MX_QUADSPI1_Init();

	if (QSPI_ResetChip() != HAL_OK) {
		return HAL_ERROR;
	}

	HAL_Delay(1);

	if (QSPI_AutoPollingMemReady() != HAL_OK) {
		return HAL_ERROR;
	}

	if (QSPI_WriteEnable() != HAL_OK) {

		return HAL_ERROR;
	}

	if (QSPI_Configuration() != HAL_OK) {
		return HAL_ERROR;
	}

	HAL_Delay(100);

	return HAL_OK;

}

uint8_t CSP_QSPI_Erase_Chip(void) {

	QSPI_CommandTypeDef sCommand;

	if (QSPI_WriteEnable() != HAL_OK) {
		return HAL_ERROR;
	}

	/* Erasing Sequence --------------------------------- */
	sCommand.InstructionMode = QSPI_INSTRUCTION_1_LINE;
	sCommand.Instruction = CHIP_ERASE_CMD;
	sCommand.AddressMode = QSPI_ADDRESS_NONE;
	sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	sCommand.DataMode = QSPI_DATA_NONE;
	sCommand.DummyCycles = 0;
	sCommand.DdrMode = QSPI_DDR_MODE_DISABLE;
	sCommand.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
	sCommand.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

	if (HAL_QSPI_Command(&hqspi1, &sCommand, HAL_MAX_DELAY) != HAL_OK) {
		return HAL_ERROR;
	}

	if (QSPI_AutoPollingMemReady() != HAL_OK) {
		return HAL_ERROR;
	}

	return HAL_OK;

}

uint8_t QSPI_AutoPollingMemReady(void) {

	QSPI_CommandTypeDef sCommand = { 0 };
	QSPI_AutoPollingTypeDef sConfig = { 0 };
	HAL_StatusTypeDef ret;

	/* Configure automatic polling mode to wait for memory ready ------ */
	sCommand.InstructionMode = QSPI_INSTRUCTION_1_LINE;
	sCommand.Instruction = READ_STATUS_REG_CMD;
	sCommand.AddressMode = QSPI_ADDRESS_NONE;
	sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	sCommand.DataMode = QSPI_DATA_1_LINE;
	sCommand.DummyCycles = 0;
	sCommand.DdrMode = QSPI_DDR_MODE_DISABLE;
	sCommand.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
	sCommand.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

	sConfig.Match = 0x00;
	sConfig.Mask = 0x01;
	sConfig.MatchMode = QSPI_MATCH_MODE_AND;
	sConfig.StatusBytesSize = 1;
	sConfig.Interval = 0x10;
	sConfig.AutomaticStop = QSPI_AUTOMATIC_STOP_ENABLE;
	if ((ret = HAL_QSPI_AutoPolling(&hqspi1, &sCommand, &sConfig,
			HAL_MAX_DELAY)) != HAL_OK) {
		return ret;
	}
	return HAL_OK;
}

static uint8_t QSPI_WriteEnable(void) {
	QSPI_CommandTypeDef sCommand = { 0 };
	QSPI_AutoPollingTypeDef sConfig = { 0 };
	HAL_StatusTypeDef ret;

	/* Enable write operations ------------------------------------------ */
	sCommand.InstructionMode = QSPI_INSTRUCTION_1_LINE;
	sCommand.Instruction = WRITE_ENABLE_CMD;
	sCommand.AddressMode = QSPI_ADDRESS_NONE;
	sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	sCommand.DataMode = QSPI_DATA_NONE;
	sCommand.DummyCycles = 0;
	sCommand.DdrMode = QSPI_DDR_MODE_DISABLE;
	sCommand.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
	sCommand.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

	if ((ret = HAL_QSPI_Command(&hqspi1, &sCommand,
			HAL_QSPI_TIMEOUT_DEFAULT_VALUE)) != HAL_OK) {
		return ret;
	}

	/* Configure automatic polling mode to wait for write enabling ---- */
	sConfig.Match = 0x02;
	sConfig.Mask = 0x02;
	sConfig.MatchMode = QSPI_MATCH_MODE_AND;
	sConfig.StatusBytesSize = 1;
	sConfig.Interval = 0x10;
	sConfig.AutomaticStop = QSPI_AUTOMATIC_STOP_ENABLE;

	sCommand.Instruction = READ_STATUS_REG_CMD;
	sCommand.DataMode = QSPI_DATA_1_LINE;

	if ((ret = HAL_QSPI_AutoPolling(&hqspi1, &sCommand, &sConfig,
			HAL_QSPI_TIMEOUT_DEFAULT_VALUE)) != HAL_OK) {
		return ret;
	}
	return HAL_OK;
}
/*Enable quad mode*/
/* Las versiones acabadas en IQ lo llevan activado por defecto, como en mi caso. */

uint8_t QSPI_Configuration(void) {

	QSPI_CommandTypeDef sCommand = { 0 };
	uint8_t reg;
	HAL_StatusTypeDef ret;

	/* Read Volatile Configuration register 2 --------------------------- */
	sCommand.InstructionMode = QSPI_INSTRUCTION_1_LINE;
	sCommand.Instruction = READ_STATUS_REG2_CMD;
	sCommand.AddressMode = QSPI_ADDRESS_NONE;
	sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	sCommand.DataMode = QSPI_DATA_1_LINE;
	sCommand.DummyCycles = 0;
	sCommand.DdrMode = QSPI_DDR_MODE_DISABLE;
	sCommand.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
	sCommand.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
	sCommand.NbData = 1;

	if ((ret = HAL_QSPI_Command(&hqspi1, &sCommand,
			HAL_QSPI_TIMEOUT_DEFAULT_VALUE)) != HAL_OK) {
		return ret;
	}

	if ((ret = HAL_QSPI_Receive(&hqspi1, &reg, HAL_QSPI_TIMEOUT_DEFAULT_VALUE))
			!= HAL_OK) {
		return ret;
	}

	/* Enable Volatile Write operations ---------------------------------------- */
	sCommand.DataMode = QSPI_DATA_NONE;
	sCommand.Instruction = VOLATILE_SR_WRITE_ENABLE;

	if (HAL_QSPI_Command(&hqspi1, &sCommand, HAL_QSPI_TIMEOUT_DEFAULT_VALUE)
			!= HAL_OK) {
		return ret;
	}

	/* Write Volatile Configuration register 2 (QE = 1) -- */
	sCommand.DataMode = QSPI_DATA_1_LINE;
	sCommand.Instruction = WRITE_STATUS_REG2_CMD;
	reg |= 2; // QE bit

	if (HAL_QSPI_Command(&hqspi1, &sCommand, HAL_QSPI_TIMEOUT_DEFAULT_VALUE)
			!= HAL_OK) {
		return ret;
	}

	if (HAL_QSPI_Transmit(&hqspi1, &reg, HAL_QSPI_TIMEOUT_DEFAULT_VALUE)
			!= HAL_OK) {
		return ret;
	}

	/* Read Volatile Configuration register 3 --------------------------- */
	sCommand.InstructionMode = QSPI_INSTRUCTION_1_LINE;
	sCommand.Instruction = READ_STATUS_REG3_CMD;
	sCommand.AddressMode = QSPI_ADDRESS_NONE;
	sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	sCommand.DataMode = QSPI_DATA_1_LINE;
	sCommand.DummyCycles = 0;
	sCommand.DdrMode = QSPI_DDR_MODE_DISABLE;
	sCommand.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
	sCommand.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
	sCommand.NbData = 1;

	if ((ret = HAL_QSPI_Command(&hqspi1, &sCommand,
			HAL_QSPI_TIMEOUT_DEFAULT_VALUE)) != HAL_OK) {
		return ret;
	}

	if ((ret = HAL_QSPI_Receive(&hqspi1, &reg, HAL_QSPI_TIMEOUT_DEFAULT_VALUE))
			!= HAL_OK) {
		return ret;
	}

	/* Write Volatile Configuration register 2 (DRV1:2 = 00) -- */
	sCommand.Instruction = WRITE_STATUS_REG3_CMD;
	reg &= 0x9f; // DRV1:2 bit

	if (HAL_QSPI_Command(&hqspi1, &sCommand, HAL_QSPI_TIMEOUT_DEFAULT_VALUE)
			!= HAL_OK) {
		return ret;
	}

	if (HAL_QSPI_Transmit(&hqspi1, &reg, HAL_QSPI_TIMEOUT_DEFAULT_VALUE)
			!= HAL_OK) {
		return ret;
	}

	return HAL_OK;
}

uint8_t CSP_QSPI_EraseBlock(uint32_t flash_address) {
	QSPI_CommandTypeDef sCommand = { 0 };
	HAL_StatusTypeDef ret;

	sCommand.InstructionMode = QSPI_INSTRUCTION_1_LINE;
	sCommand.AddressSize = QSPI_ADDRESS_24_BITS;
	sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	sCommand.DdrMode = QSPI_DDR_MODE_DISABLE;
	sCommand.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
	sCommand.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

	/* Enable write operations ------------------------------------------- */
	if ((ret = QSPI_WriteEnable()) != HAL_OK) {
		return ret;
	}

	/* Erasing Sequence -------------------------------------------------- */
	sCommand.Instruction = BLOCK_ERASE_CMD;
	sCommand.AddressMode = QSPI_ADDRESS_1_LINE;
	sCommand.Address = flash_address;
	sCommand.DataMode = QSPI_DATA_NONE;
	sCommand.DummyCycles = 0;

	if ((ret = HAL_QSPI_Command(&hqspi1, &sCommand,
			HAL_QSPI_TIMEOUT_DEFAULT_VALUE)) != HAL_OK) {
		return ret;
	}
	return HAL_OK;
}

uint8_t CSP_QSPI_EraseSector(uint32_t EraseStartAddress,
		uint32_t EraseEndAddress) {

	QSPI_CommandTypeDef sCommand;

	EraseStartAddress = EraseStartAddress
			- EraseStartAddress % MEMORY_SECTOR_SIZE;

	/* Erasing Sequence -------------------------------------------------- */
	sCommand.InstructionMode = QSPI_INSTRUCTION_1_LINE;
	sCommand.Instruction = SECTOR_ERASE_CMD;
	sCommand.AddressMode = QSPI_ADDRESS_1_LINE;
	sCommand.AddressSize = QSPI_ADDRESS_24_BITS;
	sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	sCommand.DdrMode = QSPI_DDR_MODE_DISABLE;
	sCommand.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
	sCommand.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

	sCommand.DataMode = QSPI_DATA_NONE;
	sCommand.DummyCycles = 0;

	while (EraseEndAddress >= EraseStartAddress) {
		sCommand.Address = (EraseStartAddress & 0x0FFFFFFF);

		if (QSPI_WriteEnable() != HAL_OK) {
			return HAL_ERROR;
		}

		if (HAL_QSPI_Command(&hqspi1, &sCommand, HAL_QSPI_TIMEOUT_DEFAULT_VALUE)
				!= HAL_OK) {
			return HAL_ERROR;
		}
		EraseStartAddress += MEMORY_SECTOR_SIZE;

		if (QSPI_AutoPollingMemReady() != HAL_OK) {
			return HAL_ERROR;
		}
	}

	return HAL_OK;
}

uint8_t CSP_QSPI_WriteMemory(uint8_t *buffer, uint32_t address,
		uint32_t buffer_size) {

	QSPI_CommandTypeDef sCommand;
	uint32_t end_addr, current_size, current_addr;

	/* Calculation of the size between the write address and the end of the page */
	current_addr = 0;

	//
	while (current_addr <= address) {
		current_addr += MEMORY_PAGE_SIZE;
	}
	current_size = current_addr - address;

	/* Check if the size of the data is less than the remaining place in the page */
	if (current_size > buffer_size) {
		current_size = buffer_size;
	}

	/* Initialize the adress variables */
	current_addr = address;
	end_addr = address + buffer_size;

	sCommand.InstructionMode = QSPI_INSTRUCTION_1_LINE;
	sCommand.Instruction = QUAD_IN_FAST_PROG_CMD;
	sCommand.AddressMode = QSPI_ADDRESS_1_LINE;
	sCommand.AddressSize = QSPI_ADDRESS_24_BITS;
	sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	sCommand.DdrMode = QSPI_DDR_MODE_DISABLE;
	sCommand.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
	sCommand.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
	sCommand.DataMode = QSPI_DATA_4_LINES;
	sCommand.NbData = buffer_size;
	sCommand.Address = address;
	sCommand.DummyCycles = 0;

	/* Perform the write page by page */
	do {
		sCommand.Address = current_addr;
		sCommand.NbData = current_size;

		if (current_size == 0) {
			return HAL_OK;
		}

		/* Enable write operations */
		if (QSPI_WriteEnable() != HAL_OK) {
			return HAL_ERROR;
		}

		/* Configure the command */
		if (HAL_QSPI_Command(&hqspi1, &sCommand, HAL_QSPI_TIMEOUT_DEFAULT_VALUE)
				!= HAL_OK) {

			return HAL_ERROR;
		}

		/* Transmission of the data */
		if (HAL_QSPI_Transmit(&hqspi1, buffer, HAL_QSPI_TIMEOUT_DEFAULT_VALUE)
				!= HAL_OK) {

			return HAL_ERROR;
		}

		/* Configure automatic polling mode to wait for end of program */
		if (QSPI_AutoPollingMemReady() != HAL_OK) {
			return HAL_ERROR;
		}

		/* Update the address and size variables for next page programming */
		current_addr += current_size;
		buffer += current_size;
		current_size =
				((current_addr + MEMORY_PAGE_SIZE) > end_addr) ?
						(end_addr - current_addr) : MEMORY_PAGE_SIZE;
	} while (current_addr <= end_addr);

	return HAL_OK;

}

//uint8_t CSP_QSPI_EnableMemoryMappedMode(void) {
//
//	QSPI_CommandTypeDef sCommand;
//	QSPI_MemoryMappedTypeDef sMemMappedCfg;
//
//	/* Enable Memory-Mapped mode-------------------------------------------------- */
//
//	sCommand.InstructionMode = QSPI_INSTRUCTION_1_LINE;
//	sCommand.Instruction = QUAD_OUT_FAST_READ_CMD;
//	sCommand.AddressSize = QSPI_ADDRESS_24_BITS;
//	sCommand.AddressMode = QSPI_ADDRESS_1_LINE;
//	sCommand.Address = 0;
//	sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
//	sCommand.DdrMode = QSPI_DDR_MODE_DISABLE;
//	sCommand.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
//	sCommand.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
//	sCommand.DataMode = QSPI_DATA_4_LINES;
//	sCommand.NbData = 0;
//	sCommand.DummyCycles = 4;
//	sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
//	sCommand.AlternateBytes = 0;
//	sCommand.AlternateBytesSize = 0;
//
//	sMemMappedCfg.TimeOutActivation = QSPI_TIMEOUT_COUNTER_DISABLE;
//	sMemMappedCfg.TimeOutPeriod = 0;
//
//	if (HAL_QSPI_MemoryMapped(&hqspi1, &sCommand, &sMemMappedCfg) != HAL_OK) {
//		return HAL_ERROR;
//	}
//	return HAL_OK;
//}

uint8_t CSP_QSPI_EnableMemoryMappedMode(void) {

	QSPI_CommandTypeDef sCommand;
	QSPI_MemoryMappedTypeDef sMemMappedCfg;

	/* Enable Memory-Mapped mode-------------------------------------------------- */

	sCommand.InstructionMode = QSPI_INSTRUCTION_1_LINE;
	sCommand.Instruction = QUAD_IN_OUT_FAST_READ_CMD;
	sCommand.AddressSize = QSPI_ADDRESS_24_BITS;
	sCommand.AddressMode = QSPI_ADDRESS_4_LINES;
	sCommand.Address = 0;
	sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_4_LINES;
	sCommand.AlternateBytes = 0xFF;
	sCommand.AlternateBytesSize = 1;
	sCommand.DdrMode = QSPI_DDR_MODE_DISABLE;
	sCommand.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
	sCommand.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
	sCommand.DataMode = QSPI_DATA_4_LINES;
	sCommand.NbData = 0;
	sCommand.DummyCycles = 4;

	sMemMappedCfg.TimeOutActivation = QSPI_TIMEOUT_COUNTER_DISABLE;
	sMemMappedCfg.TimeOutPeriod = 0;

	if (HAL_QSPI_MemoryMapped(&hqspi1, &sCommand, &sMemMappedCfg) != HAL_OK) {
		return HAL_ERROR;
	}
	HAL_Delay(100);
	return HAL_OK;
}

uint8_t QSPI_ResetChip() {
	QSPI_CommandTypeDef sCommand = { 0 };
	uint32_t temp = 0;
	HAL_StatusTypeDef ret;

	/* Enable Reset --------------------------- */
	sCommand.InstructionMode = QSPI_INSTRUCTION_1_LINE;
	sCommand.Instruction = RESET_ENABLE_CMD;
	sCommand.AddressMode = QSPI_ADDRESS_NONE;
	sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	sCommand.DataMode = QSPI_DATA_NONE;
	sCommand.DummyCycles = 0;
	sCommand.DdrMode = QSPI_DDR_MODE_DISABLE;
	sCommand.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
	sCommand.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

	if ((ret = HAL_QSPI_Command(&hqspi1, &sCommand,
			HAL_QSPI_TIMEOUT_DEFAULT_VALUE)) != HAL_OK) {
		return ret;
	}

	/* Reset Device --------------------------- */
	sCommand.InstructionMode = QSPI_INSTRUCTION_1_LINE;
	sCommand.Instruction = RESET_EXECUTE_CMD;
	sCommand.AddressMode = QSPI_ADDRESS_NONE;
	sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	sCommand.DataMode = QSPI_DATA_NONE;
	sCommand.DummyCycles = 0;
	sCommand.DdrMode = QSPI_DDR_MODE_DISABLE;
	sCommand.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
	sCommand.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

	if ((ret = HAL_QSPI_Command(&hqspi1, &sCommand,
			HAL_QSPI_TIMEOUT_DEFAULT_VALUE)) != HAL_OK) {
		return ret;
	}

	for (temp = 0; temp < 500000; temp++) {
		__NOP();
	}

	return HAL_OK;
}

uint8_t CSP_QSPI_Read(uint8_t *pData, uint32_t ReadAddr, uint32_t Size) {
	QSPI_CommandTypeDef s_command;

	/* Initialize the read command */
	s_command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
	s_command.Instruction = QUAD_IN_OUT_FAST_READ_CMD;
	s_command.AddressMode = QSPI_ADDRESS_4_LINES;
	s_command.AddressSize = QSPI_ADDRESS_24_BITS;
	s_command.Address = ReadAddr;
	s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	s_command.DataMode = QSPI_DATA_4_LINES;
	s_command.DummyCycles = 6U;
	s_command.NbData = Size;
	s_command.DdrMode = QSPI_DDR_MODE_DISABLE;
	s_command.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
	s_command.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

	/* Configure the command */
	if (HAL_QSPI_Command(&hqspi1, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE)
			!= HAL_OK) {
		return HAL_ERROR;
	}

	/* Set S# timing for Read command */
	MODIFY_REG(hqspi1.Instance->DCR, QUADSPI_DCR_CSHT,
			QSPI_CS_HIGH_TIME_5_CYCLE);

	/* Reception of the data */
	if (HAL_QSPI_Receive(&hqspi1, pData, HAL_QSPI_TIMEOUT_DEFAULT_VALUE)
			!= HAL_OK) {
		return HAL_ERROR;
	}

	/* Restore S# timing for nonRead commands */
	MODIFY_REG(hqspi1.Instance->DCR, QUADSPI_DCR_CSHT,
			QSPI_CS_HIGH_TIME_6_CYCLE);

	return HAL_OK;
}
/* USER CODE END 1 */
