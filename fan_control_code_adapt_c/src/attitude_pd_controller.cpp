//
// Created by msmlab on 2025/11/14.
//

#include "attitude_pd_controller.h"

#include <optional>

using Vec3 = Eigen::Vector3d;

// 姿态控制器：外环（角度）+ 内环（角速度）的串级 PID，最终输出推力器三轴力矩。
AttitudePDController::AttitudePDController(GyroScope& gyro, Fan& fan, Wheel& wheel, msmserial::MsMSerial& msm_serial):
    gyro_(gyro),
    fan_(fan),
    wheel_(wheel),
    ser_(msm_serial)
{
    std::cout << "[AttitudePDController] init" << std::endl;

    dt_ = 0.02; // 50 Hz

    angleTarget_ = Vec3::Zero();
    torque_x = 0.0;
    torque_y = 0.0;
    torque_z = 0.0;

    if_finish_balancing_ = false;

    cnt_ = 0;

    /* PID输出值初始化 */
    angle_pid_.Dout = Vec3(0.0, 0, 0);
    angle_pid_.Pout = Vec3(0, 0, 0);
    angle_pid_.Iout = Vec3(0, 0, 0);
    angle_pid_.last_error = Vec3(0, 0, 0);
    angle_pid_.out = Vec3(0, 0, 0);
    v_pid_.Dout = Vec3(0, 0, 0);
    v_pid_.Pout = Vec3(0, 0, 0);
    v_pid_.Iout = Vec3(0, 0, 0);
    v_pid_.last_error = Vec3(0, 0, 0);
    v_pid_.out = Vec3(0, 0, 0);

    /* PID上限阈值 */
    angle_pid_.max_i_out = Vec3(0, 0, 0);
    angle_pid_.max_out = Vec3(1, 1, 3);
    v_pid_.max_i_out = Vec3(50, 30, 100);
    v_pid_.max_out = Vec3(600, 600, 600);

    /* PID参数 */
    angle_pid_.Kp = Vec3(0.8, 0.8, 0.8);
    angle_pid_.Ki = Vec3(0, 0, 0);
    angle_pid_.Kd = Vec3(30, 45, 45);

    v_pid_.Kp = Vec3(180, 240, 200);
    v_pid_.Ki = Vec3(0.2, 0.8, 0.5);
    v_pid_.Kd = Vec3(0, 0, 0);
}

// 通用三轴 PID 计算：set-ref 作为误差，向量化计算 P/I/D，并执行 I 项与总输出限幅。
PID AttitudePDController::computeControl(PID& pid, Vec3& ref, Vec3& set)
{
    Vec3 error = set - ref;

    pid.Pout = pid.Kp.array() * error.array();
    pid.Iout.array() += pid.Ki.array() * error.array();
    if (pid.max_i_out[0] != 0 && pid.max_i_out[1] != 0 && pid.max_i_out[2] != 0) {
        pid.Iout[0] = std::min(pid.Iout[0], pid.max_i_out[0]);
        pid.Iout[1] = std::min(pid.Iout[1], pid.max_i_out[1]);
        pid.Iout[2] = std::min(pid.Iout[2], pid.max_i_out[2]);
        pid.Iout[0] = std::max(pid.Iout[0], -pid.max_i_out[0]);
        pid.Iout[1] = std::max(pid.Iout[1], -pid.max_i_out[1]);
        pid.Iout[2] = std::max(pid.Iout[2], -pid.max_i_out[2]);
    }
    pid.Dout = pid.Kd.array() * (error.array() - pid.last_error.array());
    pid.out = pid.Pout + pid.Iout + pid.Dout;
    pid.out[0] = std::min(pid.out[0], pid.max_out[0]);
    pid.out[1] = std::min(pid.out[1], pid.max_out[1]);
    pid.out[2] = std::min(pid.out[2], pid.max_out[2]);
    pid.out[0] = std::max(pid.out[0], -pid.max_out[0]);
    pid.out[1] = std::max(pid.out[1], -pid.max_out[1]);
    pid.out[2] = std::max(pid.out[2], -pid.max_out[2]);
    // std::cout << "pid.out: " << pid.out[0] << ", " << pid.out[1] << ", " << pid.out[2] << std::endl;

    pid.last_error[0] = error[0];
    pid.last_error[1] = error[1];
    pid.last_error[2] = error[2];

    return pid;
}

void AttitudePDController::setAttitudeInBalancing(const Vec3& eulerAngleDeg,
    const std::optional<GyroScope::Vec3> other_av,
    const std::optional<GyroScope::Vec3> other_at)
{
    // 调平/姿态控制（角度串级）：角度误差 -> 角速度指令 -> 力矩输出。

    angleTarget_ = eulerAngleDeg;

    // 读当前角速度与姿态（deg/s 与 deg）
    auto av = other_av.value_or(gyro_.getAngularVelocity());
    auto at = other_at.value_or(gyro_.getAttitude());
    Vec3 angleCurrentDeg(at.x, at.y, at.z);
    Vec3 wCurrentDeg(av.x, av.y, av.z);

    // 外环：角度 -> 角速度指令
    PID wCmd = computeControl(angle_pid_, angleCurrentDeg, angleTarget_);

    // 轴系方向约定：保持 zheda 的 yaw 方向（与历史版本一致）
    wCmd.out[2] = -wCmd.out[2];

    // 内环：角速度 -> 力矩输出
    PID tauCmd = computeControl(v_pid_, wCurrentDeg, wCmd.out);
    torque_x = tauCmd.out[0];
    torque_y = tauCmd.out[1];
    torque_z = tauCmd.out[2];

    // 下发力矩（推力器）
    fan_.sendTorque(torque_x, torque_y, torque_z);

    if (if_finish_balancing_) {
        cnt_++;
        if (cnt_ >= 50) {
            cnt_ = 0;
            WheelInit _wheelinit{
            .device_id = 0x5A,
            .target_roll = (int16_t)angleTarget_.x(),
            .target_pitch = (int16_t)angleTarget_.y(),
            .target_yaw = (int16_t)angleTarget_.z(),
            .flag_balance = (uint8_t)if_finish_balancing_,
        };
            ser_.write(0x10, _wheelinit);
        }
    }
}

void AttitudePDController::setAngularVelocityInControl(const Vec3& wTargetDeg)
{
    // 读当前角速度（deg/s）
    auto av = gyro_.getAngularVelocity();
    Vec3 wCurrentDeg(av.x, av.y, av.z);

    // 与角度串级保持一致的 yaw 方向约定：z 轴目标角速度同样取反
    Vec3 wTargetAdj = wTargetDeg;
    wTargetAdj[2] = -wTargetAdj[2];

    // 只走速度环：wCurrent -> wTargetAdj，输出力矩
    PID tauCmd = computeControl(v_pid_, wCurrentDeg, wTargetAdj);

    torque_x = tauCmd.out[0];
    torque_y = tauCmd.out[1];
    torque_z = tauCmd.out[2];

    fan_.sendTorque(torque_x, torque_y, torque_z);
}



Vec3 AttitudePDController::getTorque()
{
    Vec3 current_torque;
    current_torque.x() = torque_x;
    current_torque.y() = torque_y;
    current_torque.z() = torque_z;
    return current_torque;
}

Vec3 AttitudePDController::getCurrentGyroAttitude()
{
    auto at = gyro_.getAttitude();
    return Vec3(at.x, at.y, at.z);
}

void AttitudePDController::setIfFinishBalancing(bool if_finish_balancing)
{
    if_finish_balancing_ = if_finish_balancing;
}
