//
// Created by msmlab on 2025/11/14.
//

#ifndef FAN_CONTROL_CODE_ADAPT_C_ATTITUDE_PD_CONTROLLER_H
#define FAN_CONTROL_CODE_ADAPT_C_ATTITUDE_PD_CONTROLLER_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <cmath>
#include <optional>
#include "serial.h"
#include "gyro_scope.h"
#include "fan.h"
#include "wheel.h"

using Vec3 = Eigen::Vector3d;

struct PID {
    Vec3 Kp, Ki, Kd;      // PID 参数（按 roll/pitch/yaw 三轴组织）
    Vec3 Pout, Iout, Dout; // 各项输出（内部状态）
    Vec3 last_error;       // 上一次误差（用于 D 项）
    Vec3 max_i_out;        // I 项限幅（0 表示不启用）
    Vec3 max_out;          // 总输出限幅
    Vec3 out;              // 最终输出
};

class AttitudePDController {
private:
    GyroScope &gyro_;              // 陀螺仪/姿态数据源（角速度/欧拉角）
    Fan &fan_;                     // 推力器执行器（力矩模式）
    Wheel &wheel_;                 // 动量轮执行器（调平完成后通知等）
    msmserial::MsMSerial &ser_;    // 与 C 板通信的串口对象（用于下发 WheelInit 等结构体）

    double dt_;                    // 控制周期（s），用于调参/对齐控制频率

    Vec3 angleTarget_;             // 目标欧拉角（deg）

    double torque_x;               // 当前输出力矩 Tx（N·m 或内部等效单位）
    double torque_y;               // 当前输出力矩 Ty
    double torque_z;               // 当前输出力矩 Tz

    bool if_finish_balancing_;     // 是否已完成调平（用于触发 WheelInit 通知）

    int cnt_;                      // 计数器：降低 WheelInit 发送频率（避免刷屏/带宽占用）

    PID angle_pid_;                // 外环：角度误差 -> 角速度指令（deg/s）
    PID v_pid_;                    // 内环：角速度误差 -> 力矩指令

public:
    explicit AttitudePDController(GyroScope &gyro, Fan &fan, Wheel &wheel, msmserial::MsMSerial &msm_serial);

    // 调平/姿态控制接口：输入目标欧拉角（deg），输出并下发推力器力矩指令。
    // other_av/other_at 用于外部注入“替代测量值”（例如动捕融合/回放），为空则读取 gyro_ 的最新值。
    void setAttitudeInBalancing(const Vec3 &eulerAngleDeg,
                                std::optional<GyroScope::Vec3> other_av = std::nullopt,
                                std::optional<GyroScope::Vec3> other_at = std::nullopt);

    // 速度控制接口：仅走角速度内环（deg/s -> torque），用于特定模式下的姿态角速度控制。
    void setAngularVelocityInControl(const Vec3 &wTargetDeg);

    // 通用 PID 计算（按三轴向量化）：ref 为当前值，set 为目标值，返回更新后的 pid（含 out）。
    PID computeControl(PID &pid, Vec3 &ref, Vec3 &set);

    // 读取当前下发的力矩输出（与 fan_.sendTorque 保持一致）。
    Vec3 getTorque();

    // 读取当前陀螺仪姿态（deg）。
    Vec3 getCurrentGyroAttitude();

    // 设置调平完成标志：true 时会周期性发送 WheelInit 通知。
    void setIfFinishBalancing(bool if_finish_balancing);
};

#endif //FAN_CONTROL_CODE_ADAPT_C_ATTITUDE_PD_CONTROLLER_H
