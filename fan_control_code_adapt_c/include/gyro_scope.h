//
// Created by msmlab on 2025/11/14.
//

#ifndef FAN_CONTROL_CODE_ADAPT_C_GYRO_SCOPE_H
#define FAN_CONTROL_CODE_ADAPT_C_GYRO_SCOPE_H

#include <vector>
#include <mutex>
#include <cstring>
#include <algorithm>
#include <chrono>
#include <thread>
#include "serial.h"

class GyroScope {
public:
    /* 构造：传入已打开的串口对象 */
    explicit GyroScope(msmserial::MsMSerial &msm_serial);

    struct Vec3
    {
        double x{}, y{}, z{};
        Vec3() = default;
        Vec3(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {}
    };

    /// 读取最新角速度
    [[nodiscard]] Vec3 getAngularVelocity() const { return latestAngularVel_; }
    /// 读取最新姿态角
    [[nodiscard]] Vec3 getAttitude() const { return latestAttitude_; }
    /// 设定当前角速度
    void setAngularVelocity(double wx, double wy, double wz);
    /// 设定当前姿态角
    void setAttitude(double roll, double pitch, double yaw);

private:
    msmserial::MsMSerial ser_;

    Vec3 latestAngularVel_{};           // 角速度
    Vec3 latestAttitude_{};             // 姿态角
};

#endif //FAN_CONTROL_CODE_ADAPT_C_GYRO_SCOPE_H