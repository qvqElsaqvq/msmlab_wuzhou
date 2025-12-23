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

    start_toq_ = 50;

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

    wheel_pid_x_.Dout = Vec3(0, 0, 0);
    wheel_pid_x_.Pout = Vec3(0, 0, 0);
    wheel_pid_x_.Iout = Vec3(0, 0, 0);
    wheel_pid_x_.last_error = Vec3(0, 0, 0);
    wheel_pid_x_.out = Vec3(0, 0, 0);

    wheel_pid_y_.Dout = Vec3(0, 0, 0);
    wheel_pid_y_.Pout = Vec3(0, 0, 0);
    wheel_pid_y_.Iout = Vec3(0, 0, 0);
    wheel_pid_y_.last_error = Vec3(0, 0, 0);
    wheel_pid_y_.out = Vec3(0, 0, 0);

    wheel_pid_z_.Dout = Vec3(0, 0, 0);
    wheel_pid_z_.Pout = Vec3(0, 0, 0);
    wheel_pid_z_.Iout = Vec3(0, 0, 0);
    wheel_pid_z_.last_error = Vec3(0, 0, 0);
    wheel_pid_z_.out = Vec3(0, 0, 0);

    /* PID上限阈值 */
    angle_pid_.max_i_out = Vec3(0, 0, 0);
    angle_pid_.max_out = Vec3(1, 1, 3);
    v_pid_.max_i_out = Vec3(50, 50, 100);
    v_pid_.max_out = Vec3(600, 600, 600);

    wheel_pid_x_.max_i_out = Vec3(0, 0, 0);
    wheel_pid_x_.max_out = Vec3(3000, 0, 0);
    wheel_pid_y_.max_i_out = Vec3(0, 0, 0);
    wheel_pid_y_.max_out = Vec3(3000, 0, 0);
    wheel_pid_z_.max_i_out = Vec3(0, 0, 0);
    wheel_pid_z_.max_out = Vec3(3000, 0, 0);

    /* PID参数 */
    angle_pid_.Kp = Vec3(0.8, 0.8, 0.8);
    angle_pid_.Ki = Vec3(0, 0, 0);
    angle_pid_.Kd = Vec3(30, 15, 45);

    v_pid_.Kp = Vec3(180, 175, 200);
    v_pid_.Ki = Vec3(0.2, 0.4, 0.5);
    v_pid_.Kd = Vec3(0, 0, 0);

    wheel_pid_x_.Kp = Vec3(6000, 0, 0);
    wheel_pid_x_.Ki = Vec3(0.0, 0.0, 0.0);
    wheel_pid_x_.Kd = Vec3(5000, 0, 0);

    wheel_pid_y_.Kp = Vec3(6000, 0, 0);
    wheel_pid_y_.Ki = Vec3(0.0, 0.0, 0.0);
    wheel_pid_y_.Kd = Vec3(5000, 0, 0);

    wheel_pid_z_.Kp = Vec3(6000, 0, 0);
    wheel_pid_z_.Ki = Vec3(0.0, 0.0, 0.0);
    wheel_pid_z_.Kd = Vec3(5000, 0, 0);
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
    Vec3 wCurrentDeg(av.x, av.z, av.y);

    /* X/Y/Z 双环 PID */
    PID wCmd = computeControl(angle_pid_, angleCurrentDeg, angleTarget_);
    wCmd.out[1] = -wCmd.out[1];
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

    /* 动量轮参与控制 */
    if(if_finish_balancing_)
    {
        if(fabs(angleTarget_.x() - at.x) < 0.5)
        {
            PID pid_x = computeControl(wheel_pid_x_, angleCurrentDeg, angleTarget_);
            uint8_t dir;
            if(pid_x.out[0] > 0)
            {
                pid_x.out[0] += start_toq_;
                dir = 0x55;
            }
            else if(pid_x.out[0] < 0)
            {
                pid_x.out[0] -= start_toq_;
                dir = 0xAA;
            }
            else
            {
                pid_x.out[0] = 0;
                dir = 0x55;
            }
            wheel_.sendFrame(0x03, dir, pid_x.out[0]);
        }
        if(fabs(angleTarget_.y() - at.y) < 0.5)
        {
            PID pid_y = computeControl(wheel_pid_y_, angleCurrentDeg, angleTarget_);
            uint8_t dir;
            if(pid_y.out[0] > 0)
            {
                pid_y.out[0] += start_toq_;
                dir = 0x55;
            }
            else if(pid_y.out[0] < 0)
            {
                pid_y.out[0] -= start_toq_;
                dir = 0xAA;
            }
            else
            {
                pid_y.out[0] = 0;
                dir = 0x55;
            }
            wheel_.sendFrame(0x02, dir, pid_y.out[0]);
        }
        if(fabs(angleTarget_.z() - at.z) < 0.5)
        {
            PID pid_z = computeControl(wheel_pid_z_, angleCurrentDeg, angleTarget_);
            uint8_t dir;
            if(pid_z.out[0] > 0)
            {
                pid_z.out[0] += start_toq_;
                dir = 0x55;
            }
            else if(pid_z.out[0] < 0)
            {
                pid_z.out[0] -= start_toq_;
                dir = 0xAA;
            }
            else
            {
                pid_z.out[0] = 0;
                dir = 0x55;
            }
            wheel_.sendFrame(0x01, dir, pid_z.out[0]);
        }
    }
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
