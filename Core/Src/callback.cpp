#include "can.h"
#include "main.h"
#include "pid.h"
#include "motor.h"

extern CAN_RxHeaderTypeDef RxHeader;
uint32_t TxMailbox = CAN_TX_MAILBOX0;
CAN_TxHeaderTypeDef TxHeader ={0x1FF,0,CAN_ID_STD,CAN_RTR_DATA, 8, DISABLE};
extern uint8_t rx_data[8];
extern uint8_t tx_data[8];
extern PID pid;
extern Motor motor;
extern float ref;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, rx_data);
    motor.canRxMsgCallback_v3(rx_data);
    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, tx_data, &TxMailbox);

    pid_pos.ref_ = ref;
    pid_pos.fdb_ = motor.ecd_angle_;

    float err_ = pid_pos.ref_ - pid_pos.fdb_;
    if (err_ > 180) {
        pid_pos.fdb_ += 360;
    } else if (err_ < -180) {
        pid_pos.fdb_ -= 360;
    }
    float target_speed = pid_pos.calc(pid_pos.ref_, pid_pos.fdb_);

    pid_vit.ref_ = target_speed;
    pid_vit.fdb_ = motor.rotate_speed_;

    uint16_t output = uint16_t(pid_vit.calc(pid_vit.ref_, pid_vit.fdb_));
    tx_data[0] = uint8_t(output >> 8);
    tx_data[1] = uint8_t(output & 0xFF);
}