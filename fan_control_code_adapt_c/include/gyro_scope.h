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
    explicit GyroScope(MsMSerial &msm_serial): ser_(std::move(msm_serial)) {}

    struct Vec3
    {
        double x{}, y{}, z{};
        Vec3() = default;
        Vec3(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {}
    };

    /* 非阻塞读取 + 解析，返回解析出的帧数 */
    size_t readGyro();

    /* 线程安全读取最新角速度 */
    Vec3 getAngularVelocity() const
    {
        std::lock_guard<std::mutex> lg(mtx_);
        return latestAngularVel_;
    }

    /* 线程安全读取最新姿态角 */
    Vec3 getAttitude() const
    {
        std::lock_guard<std::mutex> lg(mtx_);
        return latestAttitude_;
    }

private:
    MsMSerial ser_;

    static constexpr uint8_t HEADER[] = {0x6F, 0x6E};
    static constexpr size_t SHORT_FRAME_LEN = 16;
    static constexpr size_t MAX_PER_CALL = 8;

    std::vector<uint8_t> buf_;          // 环形缓冲
    Vec3 latestAngularVel_{};           // 角速度
    Vec3 latestAttitude_{};             // 姿态角
    mutable std::mutex mtx_;

    /* 工具：高低字节 -> int16_t（大端） */
    static int16_t bytesToShort(uint8_t high, uint8_t low)
    {
        uint16_t u = (static_cast<uint16_t>(high) << 8) | low;
        return static_cast<int16_t>(u);
    }
};

#endif //FAN_CONTROL_CODE_ADAPT_C_GYRO_SCOPE_H