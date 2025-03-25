#include "aligncan.h"

// Needs FDCAN to be configured to use the HSE, and the HSE needs to be either a 12MHz crystal, or a 20MHz crystal
Align_StatusTypedef Align_CAN_Init(FDCAN_HandleTypeDef *hfdcan, Align_CAN_Speed_Typedef can_speed, FDCAN_GlobalTypeDef *fdcan_instance)
{
    HAL_FDCAN_DeInit(hfdcan); // This overwrites earlier settings

    hfdcan->Instance = fdcan_instance;
    hfdcan->Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    hfdcan->Init.Mode = FDCAN_MODE_NORMAL;
    hfdcan->Init.AutoRetransmission = DISABLE;
    hfdcan->Init.TransmitPause = DISABLE;
    hfdcan->Init.ProtocolException = DISABLE;
    hfdcan->Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;

    // We dont currently use FDCAN
    hfdcan->Init.NominalPrescaler = 16;
    hfdcan->Init.NominalSyncJumpWidth = 1;
    hfdcan->Init.NominalTimeSeg1 = 1;
    hfdcan->Init.NominalTimeSeg2 = 1;

    switch (HSE_VALUE)
    {
    case 12000000:
        switch (can_speed)
        {
        case ALIGN_CAN_SPEED_500KBPS:
            hfdcan->Init.ClockDivider = FDCAN_CLOCK_DIV1;
            hfdcan->Init.DataPrescaler = 3;
            hfdcan->Init.DataSyncJumpWidth = 1;
            hfdcan->Init.DataTimeSeg1 = 6;
            hfdcan->Init.DataTimeSeg2 = 1;
            break;

        case ALIGN_CAN_SPEED_1MBPS:
            hfdcan->Init.ClockDivider = FDCAN_CLOCK_DIV1;
            hfdcan->Init.DataPrescaler = 1;
            hfdcan->Init.DataSyncJumpWidth = 1;
            hfdcan->Init.DataTimeSeg1 = 10;
            hfdcan->Init.DataTimeSeg2 = 1;
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
            hfdcan->Init.ClockDivider = FDCAN_CLOCK_DIV1;
            hfdcan->Init.DataPrescaler = 8;
            hfdcan->Init.DataSyncJumpWidth = 1;
            hfdcan->Init.DataTimeSeg1 = 6;
            hfdcan->Init.DataTimeSeg2 = 1;
            break;

        case ALIGN_CAN_SPEED_1MBPS:
            hfdcan->Init.ClockDivider = FDCAN_CLOCK_DIV1;
            hfdcan->Init.DataPrescaler = 2;
            hfdcan->Init.DataSyncJumpWidth = 1;
            hfdcan->Init.DataTimeSeg1 = 8;
            hfdcan->Init.DataTimeSeg2 = 1;
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

    HAL_StatusTypeDef ret = HAL_FDCAN_Init(hfdcan);
    if (ret != HAL_OK){
        while(true){
            // If this fails, theres no need to continue as its critical for most systems, and should be fixed
        }
    }


    if(HAL_FDCAN_Start(hfdcan) != HAL_OK){
        while(true){
            // If this fails, theres no need to continue as its critical for most systems, and should be fixed
        }
    }

    return ALIGN_OK;
}

Align_StatusTypedef Align_CAN_Send(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *data, uint8_t len)
{
    FDCAN_TxHeaderTypeDef txHeader;
    txHeader.Identifier = id;
    txHeader.DataLength = len;
    txHeader.TxFrameType = FDCAN_DATA_FRAME;
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch = FDCAN_BRS_OFF;
    txHeader.FDFormat = FDCAN_CLASSIC_CAN;
    txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    txHeader.MessageMarker = 0;
    txHeader.IdType = FDCAN_STANDARD_ID;

    HAL_StatusTypeDef ret = HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &txHeader, data);
    if(ret != HAL_OK){
        return ALIGN_ERROR;
    }
    return ALIGN_OK;
}

bool Align_CAN_Receive(FDCAN_HandleTypeDef *hfdcan, FDCAN_RxHeaderTypeDef *rxHeader, uint8_t *rxData)
{
    if(HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0){
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, rxHeader, rxData);
        return true;
    }else if(HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO1) > 0){
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, rxHeader, rxData);
        return true;
    }
    return false;
}
