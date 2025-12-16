//
// Created by msmlab on 2025/11/14.
//

#include "attitude_pd_controller.h"

using Vec3 = Eigen::Vector3d;
using Quat = Eigen::Quaterniond;

AttitudePDController::AttitudePDController(GyroScope& gyro, Fan& fan, Wheel& wheel):
    gyro_(gyro),
    fan_(fan),
    wheel_(wheel)
{
    std::cout << "[AttitudePDController] init" << std::endl;

    dt_ = 0.02; // 50 Hz

    angleTarget_ = Vec3::Zero();

    torque_x = 0.0;
    torque_y = 0.0;
    torque_z = 0.0;

    if_finish_balancing_ = false;

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
    v_pid_.max_i_out = Vec3(20, 20, 10);
    v_pid_.max_out = Vec3(200, 200, 200);

    /* PID参数 */
    angle_pid_.Kp = Vec3(0.8, 0.8, 0.8);
    angle_pid_.Ki = Vec3(0, 0, 0);
    angle_pid_.Kd = Vec3(30, 30, 45);

    v_pid_.Kp = Vec3(35, 35, 20);
    v_pid_.Ki = Vec3(0.06, 0.06, 0.02);
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
    PID wCmd = computeControl(angle_pid_, angleCurrentDeg, angleTarget_);
    // std::cout << "last_error: " << angle_pid_.last_error << std::endl;
    // std::cout << "angle out: " << wCmd.out[0] << ", " << wCmd.out[1] << ", " << wCmd.out[2] << std::endl;

    PID tauCmd = computeControl(v_pid_, wCurrentDeg, wCmd.out);
    torque_x = tauCmd.out[0];
    torque_y = tauCmd.out[1];
    torque_z = tauCmd.out[2];
    // std::cout << "torque_x=" << torque_x << ", torque_y=" << torque_y << ", torque_z=" << torque_z << std::endl;

    // 下发力矩
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
