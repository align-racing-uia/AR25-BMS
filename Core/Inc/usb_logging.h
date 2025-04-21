#ifndef USB_LOGGING_H
#include "bms_config.h"
#include "usbd_cdc_if.h"

#define USB_LOG_MAX_MESSAGE_LENGTH 64

typedef struct {

    float *cellVoltages;
    float *cellTemperatures;
    float *totalCurrent;
    uint8_t messageBuffer[USB_LOG_MAX_MESSAGE_LENGTH+2];
    BMS_Config_HandleTypeDef *bms_config;

} USB_Log_HandleTypeDef;

typedef enum {
    USB_LOG_MESSAGE_PRINT = 0,
    USB_LOG_MESSAGE_SEND_VOLTAGES = 1,
    USB_LOG_MESSAGE_SEND_TEMPERATURES = 2,
    USB_LOG_MESSAGE_SEND_CURRENT = 3,
    USB_LOG_MESSAGE_SEND_RAW = 4,
    USB_LOG_MESSAGE_SEND_BMS_CONFIG = 5,
    USB_LOG_MESSAGE_SEND_BMS_CONFIG_ACK = 6,
    USB_LOG_MESSAGE_SEND_BMS_CONFIG_NACK = 7,
    USB_LOG_MESSAGE_SEND_BMS_CONFIG_UPDATE = 8,
    USB_LOG_MESSAGE_SEND_BMS_CONFIG_UPDATE_ACK = 9,
    USB_LOG_MESSAGE_SEND_BMS_CONFIG_UPDATE_NACK = 10,
} USB_Log_MessageTypeDef;

typedef struct {
    uint8_t Type;
    uint8_t Length;
    uint8_t Data[USB_LOG_MAX_MESSAGE_LENGTH];
} USB_Log_TransmitDataTypeDef;

typedef struct {
    uint8_t Type;
    uint8_t Length;
    uint8_t Data[USB_LOG_MAX_MESSAGE_LENGTH];
} USB_Log_RecieveDataTypeDef;

void USB_Log_LoadCellVoltages(USB_Log_HandleTypeDef *usb_log);
void USB_Log_SendCellVoltages(USB_Log_HandleTypeDef *usb_log);
void USB_Log_LoadCellTemperatures(USB_Log_HandleTypeDef *usb_log);
void USB_Log_SendCellTemperatures(USB_Log_HandleTypeDef *usb_log);
void USB_Log_TransmitData(USB_Log_HandleTypeDef *usb_log, USB_Log_TransmitDataTypeDef *message);
void USB_Log_TryRecieveData(USB_Log_HandleTypeDef *usb_log);

#endif