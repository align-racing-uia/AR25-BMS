#include "icm.h"
#include "icm_regs.h"

void ICM_WriteReg(ICM_HandleTypeDef *hicm, uint8_t reg, uint8_t data)
{
    uint8_t addr = hicm->address<<1;
    HAL_I2C_Mem_Write(hicm->hi2c, addr, reg, 1, &data, 1, 100);
    // TODO: Verify that the write was successful
    uint8_t readback;
    // ICM_ReadReg(hicm, reg, 1, &data);
    HAL_I2C_Mem_Read(hicm->hi2c, addr, reg, 1, &readback, 1, 100);
    if (readback != data)
    {
        // Error
        while (1)
        {
        };
    }
}
void ICM_ReadReg(ICM_HandleTypeDef *hicm, uint8_t reg, uint8_t count, uint8_t *dest)
{
    // Placeholder for the register to read
    HAL_I2C_Master_Transmit(hicm->hi2c, hicm->address, &reg, 1, 100);
    HAL_I2C_Master_Receive(hicm->hi2c, hicm->address, dest, count, 100);
}