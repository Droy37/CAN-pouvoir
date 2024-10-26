#ifndef MOTOR_H
#define MOTOR_H
#include <cstdint>

class M3508_Motor{
public:
    float ratio_; // 电机减速比

    float angle_; // deg 输出端累计转动角度
    float delta_angle_; // deg 输出端新转动的角度

    float ecd_angle_; // deg 当前电机编码器角度
    float last_ecd_angle_; // deg 上次电机编码器角度
    float delta_ecd_angle_; // deg 编码器端新转动的角度

    float rotate_speed_; // dps 反馈转子转速

    float current_; // A 反馈转矩电流
    float temp_; // °C 反馈电机温度

    // 构造函数
    M3508_Motor():
        ratio_(-36),
        angle_(0.0),
        delta_angle_(0.0),
        ecd_angle_(0.0),
        last_ecd_angle_(0.0),
        delta_ecd_angle_(0.0),
        rotate_speed_(0.0),
        current_(0.0),
        temp_(25.0)
    {

    }

    // 线性映射函数
    float linearMapping(int in, int in_min, int in_max, float out_min, float out_max){
        float out;
        out = float(in - in_min) / float(in_max - in_min) * (out_max - out_min) + out_min;
        return out;
    }

    // 报文解析函数
    void canRxMsgCallback_v1(uint8_t rx_data[8]);
};

extern M3508_Motor motor;



#endif //MOTOR_H
