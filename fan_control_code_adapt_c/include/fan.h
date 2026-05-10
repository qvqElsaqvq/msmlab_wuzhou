//
// Created by msmlab on 2025/11/14.
//

#ifndef FAN_CONTROL_CODE_ADAPT_C_FAN_H
#define FAN_CONTROL_CODE_ADAPT_C_FAN_H

#include <array>
#include <algorithm>
#include <cmath>
#include "serial.h"

class Fan
{
private:
    msmserial::MsMSerial& ser_; // 串口对象（向 C 板下发推力器控制帧）

    bool if_power_off_;         // 停机标志：true 时禁止输出（安全保护）

public:
    // 构造：注册必要的串口回调（如校准回传），初始化状态
    explicit Fan(msmserial::MsMSerial &serial);

    /// 发送三轴力矩，单位 N·m，放大 100 倍并限幅 0-255
    void sendTorque(float tx, float ty, float tz);

    // 设置停机标志
    void setIfPowerOff( bool if_power_off );
};

#endif //FAN_CONTROL_CODE_ADAPT_C_FAN_H
