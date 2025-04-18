#ifndef USB_LOGGING_H
#include "bms_config.h"
#include "usbd_cdc_if.h"

typedef struct {

    float *cellVoltages;
    float *cellTemperatures;
    float *totalCurrent;
    BMS_ConfigTypeDef *bms_config;

} USB_LogFrameTypeDef;

void USB_Log_LoadCellVoltages(USB_LogFrameTypeDef *usb_log);
void USB_Log_LoadCellTemperatures(USB_LogFrameTypeDef *usb_log);
void USB_Log_TransmitData(USB_LogFrameTypeDef *usb_log);
void USB_Log_RecieveData(USB_LogFrameTypeDef *usb_log);

#endif