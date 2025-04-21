#include "usb_logging.h"


void USB_Log_LoadCellVoltages(USB_Log_HandleTypeDef *usb_log){

}
void USB_Log_LoadCellTemperatures(USB_Log_HandleTypeDef *usb_log){

}

void USB_Log_TryRecieveData(USB_Log_HandleTypeDef *usb_log){

}


void USB_Log_SendCellVoltages(USB_Log_HandleTypeDef *usb_log){
    // Send cell voltages to USB CDC
    USB_Log_TransmitDataTypeDef message;
    message.Length = usb_log->bms_config->CellsEach * sizeof(float); // Send one slave chip each time
    message.Type = USB_LOG_MESSAGE_SEND_VOLTAGES;

    // Prepare the message
    for (size_t i = 0; i < usb_log->bms_config->NumOfSlaves; i++) {

        memcpy(&message.Data, &usb_log->cellVoltages + i * usb_log->bms_config->CellsEach, sizeof(float) * usb_log->bms_config->CellsEach);
        
        USB_Log_TransmitData(usb_log, &message);

    }

}
void USB_Log_SendCellTemperatures(USB_Log_HandleTypeDef *usb_log){
    USB_Log_TransmitDataTypeDef message;
    message.Length = usb_log->bms_config->TempsEach * sizeof(float); // Send one slave chip each time
    message.Type = USB_LOG_MESSAGE_SEND_TEMPERATURES;
    // Prepare the message
    for(size_t i=0; i < usb_log->bms_config->NumOfSlaves; i++){
        memcpy(&message.Data, &usb_log->cellTemperatures + i * usb_log->bms_config->TempsEach, sizeof(float) * usb_log->bms_config->TempsEach);
        
        USB_Log_TransmitData(usb_log, &message);
    }
}

void USB_Log_TransmitData(USB_Log_HandleTypeDef *usb_log, USB_Log_TransmitDataTypeDef *message){
    // Send data to USB CDC
    CDC_Transmit_FS((uint8_t*) message, sizeof(USB_Log_TransmitDataTypeDef));

}
