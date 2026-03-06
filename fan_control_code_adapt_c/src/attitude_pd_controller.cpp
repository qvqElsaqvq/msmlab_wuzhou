//
// Created by msmlab on 2025/11/14.
//

#include "attitude_pd_controller.h"

#include <memory>
#include <optional>

using Vec3 = Eigen::Vector3d;
using Quat = Eigen::Quaterniond;

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
    angle_pid_.max_out = Vec3(1, 1, 2);
    v_pid_.max_i_out = Vec3(50, 50, 100);
    v_pid_.max_out = Vec3(600, 600, 600);

    /* PID参数 */
    angle_pid_.Kp = Vec3(0.8, 1.0, 1.0);
    angle_pid_.Ki = Vec3(0, 0, 0);
    angle_pid_.Kd = Vec3(100, 120, 160);

    v_pid_.Kp = Vec3(300, 350, 200);
    v_pid_.Ki = Vec3(1.0, 1.2, 0.8);
    v_pid_.Kd = Vec3(0, 0, 0);
}

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
    /*
     * 调平过程中使用的不进死循环的控制
     * X/Y/Z：双环PID（欧拉角）
     */

    angleTarget_ = eulerAngleDeg;

    /* 读当前姿态 */
    auto av = other_av.value_or(gyro_.getAngularVelocity()); // °/s
    auto at = other_at.value_or(gyro_.getAttitude());// °
    Vec3 angleCurrentDeg(at.x, at.y, at.z);
    Vec3 wCurrentDeg(av.x, av.y, av.z);

    /* X/Y/Z 双环 PID */
    auto angle_diff = [](double tar, double cur){return 180.0 / M_PI * atan2(sin(tar / 180.0 * M_PI - cur / 180.0 * M_PI), cos(tar / 180.0 * M_PI - cur / 180.0 * M_PI));};
    Vec3 err_angle{angle_diff(angleTarget_.x(), at.x), angle_diff(angleTarget_.y(), at.y), angle_diff(angleTarget_.z(), at.z)};
    Vec3 ZERO{0.0, 0.0, 0.0};
    // std::cout << "err: " <<angleTarget_.z() << " " << at.z << std::endl;
    PID wCmd = computeControl(angle_pid_, ZERO, err_angle);
    wCmd.out[2] = -wCmd.out[2];
    // std::cout << "last_error: " << angle_pid_.last_error << std::endl;
    // std::cout << "angle out: " << wCmd.out[0] << ", " << wCmd.out[1] << ", " << wCmd.out[2] << std::endl;

    PID tauCmd = computeControl(v_pid_, wCurrentDeg, wCmd.out);
    torque_x = tauCmd.out[0];
    torque_y = tauCmd.out[1];
    torque_z = tauCmd.out[2];
    // std::cout << "torque_x=" << torque_x << ", torque_y=" << torque_y << ", torque_z=" << torque_z << std::endl;

    // 下发力矩
    fan_.sendTorque(torque_x, torque_y, torque_z);

    if (if_finish_balancing_) {
        cnt_++;
        if (cnt_ >= 50) {
            cnt_ = 0;
            WheelInit _wheelinit{
            .device_id = 0x5A,
            .target_roll = (int16_t)(angleTarget_.x() * 100.0f),
            .target_pitch = (int16_t)(angleTarget_.y() * 100.0f),
            .target_yaw = (int16_t)(angleTarget_.z() * 100.0f),
            .flag_balance = (uint8_t)if_finish_balancing_,
        };
            ser_.write(0x10, _wheelinit);
        std::cout << "wheelinit: " << ",roll:"<< _wheelinit.target_roll << ",pitch:"<<  _wheelinit.target_pitch<< ",yaw:"<<  _wheelinit.target_yaw << std::endl;
        }
    }
}

void AttitudePDController::setAngularVelocityInControl(const Vec3& wTargetDeg)
{
    // 读当前角速度（deg/s）
    auto av = gyro_.getAngularVelocity();
    Vec3 wCurrentDeg(av.x, av.y, av.z);

    // 保持与你姿态环一致的 yaw 方向约定（你在姿态控制里对 wCmd.out[2] 做了反号）
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

void AttitudePDController::setIfFinishBalancing(bool if_finish_balancing)
{
    if_finish_balancing_ = if_finish_balancing;
}
