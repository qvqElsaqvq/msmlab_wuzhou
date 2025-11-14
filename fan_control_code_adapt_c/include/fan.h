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
    MsMSerial &ser_;

    /* 裁剪到 0…160 */
    static uint8_t clampByte(int x)
    {
        if (x < 0) return 0;
        if (x > 160) return 160;
        return static_cast<uint8_t>(x);
    }

    /* 发送 11 字节帧：FF FE 00 + 8 字节 payload + xor */
    void sendFrame(const std::array<uint8_t, 8> &payload);

public:
    explicit Fan(MsMSerial &serial) : ser_(serial) {}

    /* 接口：8 路 PWM，输入 0…160 */
    void setPWM(const std::array<uint8_t, 8> &pwm);

    /* 接口：三轴力矩，单位 N·m，内部放大 100 倍并限幅 0…255 */
    void setTorque(float tx, float ty, float tz);
};

#endif //FAN_CONTROL_CODE_ADAPT_C_FAN_H