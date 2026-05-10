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
    // 动量轮状态（来自串口回传/内部缓存）
    struct Status {
        uint8_t id = 0;    // 动量轮编号
        uint8_t dir = 0;   // 方向（协议定义：如 0x55 正向 / 0xAA 反向）
        uint8_t speed = 0.f; // 转速/速度（协议字段，单位由底层约定）
    };

    // 动量轮控制指令（下发）
    struct Control {
        uint8_t id = 0;      // 动量轮编号
        uint8_t dir = 0;     // 方向
        uint8_t current = 0.f; // 电流/占空等（协议字段，单位由底层约定）
    };

private:
    msmserial::MsMSerial& ser_; // 串口对象（负责实际收发）

    Control wheel_control_; // 最近一次下发的控制指令缓存
    Status wheel_status_;   // 最近一次收到/更新的状态缓存

    bool if_power_off_;     // 停机标志：true 时禁止输出（安全保护）

public:
    // 构造：注册串口回调，初始化内部状态
    explicit Wheel(msmserial::MsMSerial& serial);

    // 下发控制帧（按协议组包后通过 ser_ 发送）
    void sendFrame(uint8_t wheel_id, uint8_t dir, uint8_t current);

    // 更新内部状态缓存（供状态上报使用）
    void setStauts(uint8_t wheel_id, uint8_t dir, uint8_t speed);

    // 获取当前状态缓存
    Status getStatus();

    // 设置停机标志
    void setIfPowerOff( bool if_power_off );
};

#endif //FAN_CONTROL_CODE_ADAPT_C_WHEEL_H
