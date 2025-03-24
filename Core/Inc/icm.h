

#ifndef ICM_H
#define ICM_H
#include "stdint.h"
#include "icm_regs.h"
#include "i2c.h"

typedef enum
{
    ICM_GYRO_FS_250DPS = 3,
    ICM_GYRO_FS_500DPS = 2,
    ICM_GYRO_FS_1000DPS = 1,
    ICM_GYRO_FS_2000DPS = 0
} ICM_GYRO_FS;

typedef struct
{
    I2C_HandleTypeDef *hi2c;
    uint8_t address;
    ICM_GYRO_FS gyro_fs;
} ICM_HandleTypeDef;

void ICM_WriteReg(ICM_HandleTypeDef* hicm, uint8_t reg, uint8_t val);
void ICM_ReadReg(ICM_HandleTypeDef* hicm, uint8_t reg, uint8_t count, uint8_t *dest);

#endif