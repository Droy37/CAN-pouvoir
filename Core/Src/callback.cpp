#include "main.h"

extern CAN_RxHeaderTypeDef rx_header;
extern CAN_TxHeaderTypeDef tx_header;
extern uint8_t rx_data[8];
extern uint8_t tx_data[8];

float ecd_angle_;
float rotate_speed_;

float current_;
float temp_;


float linearMapping(int in, int in_min, int in_max, float out_min, float out_max){
    float out;
    out = float(in - in_min) / float(in_max - in_min) * (out_max - out_min) + out_min;
    return out;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    uint16_t ecd_angle_mid = (rx_data[0] << 8) | rx_data[1];
    ecd_angle_ = linearMapping(ecd_angle_mid, 0, 8191, 0.0, 360.0);

    uint16_t rotate_speed_mid = (rx_data[2] << 8) | rx_data[3];
    rotate_speed_ = static_cast<float> (rotate_speed_mid * 6);

    int16_t current_mid = (rx_data[4] << 8) | rx_data[5];
    current_ = linearMapping(current_mid, -16384, 16384, -20.0, 20.0);

    uint8_t temp_mid = rx_data[6];
    temp_ = static_cast<float> (temp_mid);

}

