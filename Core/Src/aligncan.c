#include "aligncan.h"

Align_Status_Typedef Align_CAN_Init(FDCAN_HandleTypeDef *hfdcan, Align_CAN_Speed_Typedef can_speed)
{
    FDCAN_InitTypeDef fdcan_init;
    fdcan_init.FrameFormat = FDCAN_FRAME_CLASSIC;
    fdcan_init.Mode = FDCAN_MODE_NORMAL;
    fdcan_init.AutoRetransmission = DISABLE;
    fdcan_init.TransmitPause = DISABLE;
    fdcan_init.ProtocolException = DISABLE;
    fdcan_init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;

    // We dont currently use FDCAN
    fdcan_init.NominalPrescaler = 1;
    fdcan_init.NominalSyncJumpWidth = 1;
    fdcan_init.NominalTimeSeg1 = 1;
    fdcan_init.NominalTimeSeg2 = 1;

    switch (HSE_VALUE)
    {
    case 12000000:
        switch (can_speed)
        {
        case ALIGN_CAN_SPEED_500KBPS:
            fdcan_init.ClockDivider = FDCAN_CLOCK_DIV1;
            fdcan_init.DataPrescaler = 3;
            fdcan_init.DataSyncJumpWidth = 1;
            fdcan_init.DataTimeSeg1 = 6;
            fdcan_init.DataTimeSeg2 = 1;
            break;

        case ALIGN_CAN_SPEED_1MBPS:
            fdcan_init.ClockDivider = FDCAN_CLOCK_DIV1;
            fdcan_init.DataPrescaler = 1;
            fdcan_init.DataSyncJumpWidth = 1;
            fdcan_init.DataTimeSeg1 = 10;
            fdcan_init.DataTimeSeg2 = 1;
            break;

        default:
            while (true)
            {
                // We dont support this can speed yet. Please implement it! :-)
            }
            break;
        }
        break;
    case 20000000:
        switch (can_speed)
        {
        case ALIGN_CAN_SPEED_500KBPS:
            fdcan_init.ClockDivider = FDCAN_CLOCK_DIV1;
            fdcan_init.DataPrescaler = 8;
            fdcan_init.DataSyncJumpWidth = 1;
            fdcan_init.DataTimeSeg1 = 6;
            fdcan_init.DataTimeSeg2 = 1;
            break;

        case ALIGN_CAN_SPEED_1MBPS:
            fdcan_init.ClockDivider = FDCAN_CLOCK_DIV1;
            fdcan_init.DataPrescaler = 2;
            fdcan_init.DataSyncJumpWidth = 1;
            fdcan_init.DataTimeSeg1 = 8;
            fdcan_init.DataTimeSeg2 = 1;
            break;

        default:
            while (true)
            {
                // We dont support this can speed yet. Please implement it! :-)
            }
            break;
        }
        break;
    default:
        while (true)
        {
            // We dont support this clock speed yet. Please implement it! :-)
        }
        break;
    }
    HAL_DeInit(); // This overwrites earlier settings
    hfdcan->Init = fdcan_init;
    if (HAL_FDCAN_Init(hfdcan) != HAL_OK){
        while(true){
            // If this fails, theres no need to continue as its critical for most systems, and should be fixed
        }
    }

    return ALIGN_OK;
}

void Align_CAN_Send(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *data, uint8_t len)
{
}

bool Align_CAN_Receive(FDCAN_HandleTypeDef *hfdcan, FDCAN_RxHeaderTypeDef *rxHeader, uint8_t *rxData)
{
    return false;
}
