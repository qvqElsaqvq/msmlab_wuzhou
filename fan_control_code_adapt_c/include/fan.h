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
    msmserial::MsMSerial& ser_;

    bool if_power_off_;

public:
    explicit Fan(msmserial::MsMSerial &serial);

    /// 发送三轴力矩，单位 N·m，放大 100 倍并限幅 0-255
    void sendTorque(float tx, float ty, float tz);

    void setIfPowerOff( bool if_power_off );
};

#endif //FAN_CONTROL_CODE_ADAPT_C_FAN_H