#include "main.h"
#include "can.h"

extern CAN_RxHeaderTypeDef RxHeader;
extern CAN_TxHeaderTypeDef TxHeader;
extern uint8_t rx_data[1];
extern uint8_t tx_data[1];
extern uint32_t TxMailbox;



void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, rx_data);

    if (rx_data[0] == 0x11) {
        HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == KEY_Pin) {
        HAL_CAN_AddTxMessage(&hcan1, &TxHeader, tx_data, &TxMailbox);
    }
}