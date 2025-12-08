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
    struct PID
    {
        Vec3 Kp, Ki, Kd;  // rpy轴PID参数
        Vec3 Pout, Iout, Dout;
        Vec3 last_error;
        Vec3 max_i_out;
        Vec3 max_out;
        Vec3 out;  // pid输出量
    };

private:
    msmserial::MsMSerial& ser_;

    Control wheel_control_;
    Status wheel_status_;

    PID wheel_pid_;

public:
    explicit Wheel(msmserial::MsMSerial& serial);

    void sendFrame(uint8_t wheel_id, uint8_t dir, uint8_t current);

    void setStauts(uint8_t wheel_id, uint8_t dir, uint8_t speed);

    Status getStatus();

    PID computePID(PID pid, Vec3& ref, Vec3& set);
};

#endif //FAN_CONTROL_CODE_ADAPT_C_WHEEL_H