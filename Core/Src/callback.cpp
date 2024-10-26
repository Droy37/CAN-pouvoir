#include "can.h"
#include "main.h"
#include "pid.h"
#include "motor.h"

extern CAN_RxHeaderTypeDef RxHeader;
uint32_t TxMailbox = CAN_TX_MAILBOX0;
CAN_TxHeaderTypeDef TxHeader ={0x1FF,0,CAN_ID_STD,CAN_RTR_DATA, 8, DISABLE};
extern uint8_t rx_data[8];
extern uint8_t tx_data[10];
extern PID pid;
extern M3508_Motor motor;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, rx_data);
    motor.canRxMsgCallback_v1(rx_data);
    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, tx_data, &TxMailbox);

    pid.fdb_ = motor.rotate_speed_;
    uint16_t output = uint16_t(pid.calc(pid.ref_, pid.fdb_));
    tx_data[0] = uint8_t(output >> 8);
    tx_data[1] = uint8_t(output & 0xFF);
}