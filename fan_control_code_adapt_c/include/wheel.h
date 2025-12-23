//
// Created by msmlab on 2025/11/14.
//

#ifndef FAN_CONTROL_CODE_ADAPT_C_WHEEL_H
#define FAN_CONTROL_CODE_ADAPT_C_WHEEL_H

#include <vector>
#include <cstdint>
#include <cstring>
#include <chrono>
#include <thread>
#include <stdexcept>
#include <string>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include "serial.h"

using Vec3 = Eigen::Vector3d;

class Wheel
{
public:
    struct Status
    {
        uint8_t id = 0;
        uint8_t dir = 0;
        uint8_t speed = 0.f;
    };
    struct Control{
        uint8_t id = 0;
        uint8_t dir = 0;
        uint8_t current = 0.f;
    };

private:
    msmserial::MsMSerial& ser_;

    Control wheel_control_;
    Status wheel_status_;

    bool if_power_off_;

public:
    explicit Wheel(msmserial::MsMSerial& serial);

    void sendFrame(uint8_t wheel_id, uint8_t dir, uint8_t current);

    void setStauts(uint8_t wheel_id, uint8_t dir, uint8_t speed);

    Status getStatus();

    void setIfPowerOff( bool if_power_off );
};

#endif //FAN_CONTROL_CODE_ADAPT_C_WHEEL_H