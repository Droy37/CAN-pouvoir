#include "motor.h"

void M3508_Motor::canRxMsgCallback_v1(uint8_t* rx_data){
    uint16_t ecd_angle_mid = (rx_data[0] << 8) | rx_data[1];
    ecd_angle_ = linearMapping(ecd_angle_mid, 0, 8191, 0.0, 360.0);

    int16_t rotate_speed_mid = (rx_data[2] << 8) | rx_data[3];
    rotate_speed_ = float(rotate_speed_mid * 6 / -ratio_);

    int16_t current_mid = (rx_data[4] << 8) | rx_data[5];
    current_ = linearMapping(current_mid, -16384, 16384, -20.0, 20.0);

    uint8_t temp_mid = rx_data[6];
    temp_ = float(temp_mid);
}

M3508_Motor motor;