//
// Created by msmlab on 2025/11/14.
//

#ifndef FAN_CONTROL_CODE_ADAPT_C_ATTITUDE_PD_CONTROLLER_H
#define FAN_CONTROL_CODE_ADAPT_C_ATTITUDE_PD_CONTROLLER_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <array>
#include <cmath>
#include <thread>
#include "serial.h"
#include "gyro_scope.h"
#include "fan.h"
#include "wheel.h"

using Vec3 = Eigen::Vector3d;

struct PID
{
    Vec3 Kp, Ki, Kd;  // rpy轴PID参数
    Vec3 Pout, Iout, Dout;
    Vec3 last_error;
    Vec3 max_i_out;
    Vec3 max_out;
    Vec3 out;  // pid输出量
};

class AttitudePDController
{
private:
    GyroScope& gyro_;
    Fan& fan_;
    Wheel& wheel_;
    msmserial::MsMSerial& ser_;

    /* PID 状态 */
    Vec3 intRate_ = Vec3::Zero();
    Vec3 prevEr_ = Vec3::Zero();
    double dt_;

    /* 目标 */
    Vec3 angleTarget_; // deg

    ///传入调平中的三轴力矩相关数值
    double torque_x;
    double torque_y;
    double torque_z;

    bool if_finish_balancing_;

    int cnt_;

    PID angle_pid_; // 角度环，外环
    PID v_pid_; // 速度环，内环

    /// deg ↔ rad
    static double deg2rad(double d) { return d * M_PI / 180.0; }
    static double rad2deg(double r) { return r * 180.0 / M_PI; }

public:
    explicit AttitudePDController(GyroScope& gyro, Fan& fan, Wheel& wheel, msmserial::MsMSerial& msm_serial);

    /// 传入目标欧拉角（ZYX，单位 度）
    void setAttitudeInBalancing(const Vec3& eulerAngleDeg);
    /// X/Y/Z 双环 PID，计算推力器力矩输出
    PID computeControl(PID& pid, Vec3& ref, Vec3& set);
    /// 供外部读取当前扭矩
    Vec3 getTorque();
    /// 记录是否已经调平完成
    void setIfFinishBalancing(bool if_finish_balancing);
};

#endif //FAN_CONTROL_CODE_ADAPT_C_ATTITUDE_PD_CONTROLLER_H