//
// Created by msmlab on 2025/11/14.
//

#include "attitude_pd_controller.h"

using Vec3 = Eigen::Vector3d;
using Quat = Eigen::Quaterniond;

AttitudePDController::AttitudePDController(GyroScope& gyro, Fan& fan): gyro_(gyro), fan_(fan)
{
    std::cout << "[AttitudePDController] init" << std::endl;

    /* ====== 增益 ====== */
    Kp_ = Vec3(4000, 4000, 3700);
    Kd_ = Vec3(0, 100, 80);

    /* 外环：角度 → 角速度参考 */
    Kp_anging_ = Vec3(0.42, 0.42, 0.0);
    Kd_anging_ = Vec3(0.04, 0.04, 0.0);

    /* 内环：角速度 */
    Kp_rating_ = Vec3(41, 41, 0);
    Kd_rating_ = Vec3(0, 0, 0);
    Ki_rating_ = Vec3(0.16, 0.16, 0.0);

    dt_ = 0.02; // 50 Hz

    angleTarget_ = Vec3::Zero();
    qTarget_ = Quat::Identity();

    torque_x = 0.0;
    torque_y = 0.0;
    torque_z = 0.0;

    angle_pid_.Dout = Vec3(0.0, 0, 0);
    angle_pid_.Pout = Vec3(0, 0, 0);
    angle_pid_.Iout = Vec3(0, 0, 0);
    angle_pid_.last_error = Vec3(0, 0, 0);
    angle_pid_.max_i_out = Vec3(0, 0, 0);
    angle_pid_.max_out = Vec3(0, 0, 0);
    angle_pid_.out = Vec3(0, 0, 0);
    v_pid_.Dout = Vec3(0, 0, 0);
    v_pid_.Pout = Vec3(0, 0, 0);
    v_pid_.Iout = Vec3(0, 0, 0);
    v_pid_.last_error = Vec3(0, 0, 0);
    v_pid_.max_i_out = Vec3(0, 0, 0);
    v_pid_.max_out = Vec3(0, 0, 0);
    v_pid_.out = Vec3(0, 0, 0);

    /* PID参数 */
    angle_pid_.Kp = Vec3(0, 0, 0);
    angle_pid_.Ki = Vec3(0, 0, 0);
    angle_pid_.Kd = Vec3(0, 0, 0);

    v_pid_.Kp = Vec3(0, 0, 0);
    v_pid_.Ki = Vec3(0, 0, 0);
    v_pid_.Kd = Vec3(0, 0, 0);
}

PID AttitudePDController::computeControl(PID pid, Vec3& ref, Vec3& set)
{
    Vec3 error = set - ref;

    pid.Pout = pid.Kp * error;
    pid.Iout += pid.Ki * error;
    pid.Iout = std::max(pid.Iout, pid.max_i_out);
    pid.Dout = pid.Kd * (error - pid.last_error);
    pid.out = pid.Pout + pid.Iout + pid.Dout;
    pid.out = std::max(pid.out, pid.max_out);

    return pid;
}

void AttitudePDController::setAttitudeInBalancing(const Vec3& eulerAngleDeg)
{
    /*
     * 调平过程中使用的不进死循环的控制
     * X/Y/Z：双环PID（欧拉角）
     */

    angleTarget_ = eulerAngleDeg;

    /* 读当前姿态 */
    auto av = gyro_.getAngularVelocity();  // °/s
    auto at = gyro_.getAttitude();  // °
    Vec3 angleCurrentDeg(at.x, at.y, at.z);
    Vec3 wCurrentDeg(av.x, av.y, av.z);

    /* X/Y/Z 双环 PID */
    PID wCmd = computeControl(angle_pid_, angleTarget_, angleCurrentDeg);
    PID tauCmd = computeControl(v_pid_, wCmd.out, wCurrentDeg);

    torque_x = tauCmd.out[0];
    torque_y = tauCmd.out[1];
    torque_z = tauCmd.out[2];
    // std::cout << "torque_x=" << torque_x << ", torque_y=" << torque_y << ", torque_z=" << torque_z << std::endl;

    // tauCmd.out[1] = 0;
    // tauCmd.out[2] = 0;
    // 软饱和
    // double tx = 0.5;
    // double ty = -0.5;
    double tx = std::tanh(tauCmd.out[0] / 120.0);
    double ty = std::tanh(tauCmd.out[1] / 120.0);
    double tz = std::tanh(tauCmd.out[2] / 300.0);
    std::cout << "tx: " << tx << ", ty: " << ty << ", tz: " << tz << std::endl;

    // 下发力矩
    fan_.sendTorque(static_cast<float>(tx), 0, 0);
    // fan_.sendTorque(static_cast<float>(tx), static_cast<float>(ty), static_cast<float>(tz));
}

Vec3 AttitudePDController::getTorque()
{
    Vec3 current_torque;
    current_torque.x() = torque_x;
    current_torque.y() = torque_y;
    current_torque.z() = torque_z;
    return current_torque;
}
