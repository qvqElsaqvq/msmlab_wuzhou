//
// Created by msmlab on 2025/11/14.
//

#ifndef FAN_CONTROL_CODE_ADAPT_C_ATTITUDE_PD_CONTROLLER_H
#define FAN_CONTROL_CODE_ADAPT_C_ATTITUDE_PD_CONTROLLER_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <array>
#include <cmath>
#include "serial.h"
#include "gyro_scope.h"
#include "fan.h"

using Vec3 = Eigen::Vector3d;
using Quat = Eigen::Quaterniond;

class AttitudePDController
{
private:
    GyroScope& gyro_;
    Fan& fan_;

    /* ====== 增益 ====== */
    Vec3 Kp_ = Vec3(4000, 4000, 3700);
    Vec3 Kd_ = Vec3(0, 100, 80);

    /* 外环：角度 → 角速度参考 */
    Vec3 Kp_anging_ = Vec3(0.42, 0.42, 0.0);
    Vec3 Kd_anging_ = Vec3(0.04, 0.04, 0.0);

    /* 内环：角速度 */
    Vec3 Kp_rating_ = Vec3(41, 41, 0);
    Vec3 Kd_rating_ = Vec3(0, 0, 0);
    Vec3 Ki_rating_ = Vec3(0.16, 0.16, 0.0);

    /* PID 状态 */
    Vec3 intRate_   = Vec3::Zero();
    Vec3 prevEr_    = Vec3::Zero();
    const double dt_ = 0.02;          // 50 Hz

    /* 目标 */
    Vec3 angleTarget_ = Vec3::Zero(); // deg
    Quat  qTarget_    = Quat::Identity();

    /* 软饱和参数 */
    const double slew_ = 8.0;

    /* 工具：deg ↔ rad */
    static double deg2rad(double d) { return d * M_PI / 180.0; }
    static double rad2deg(double r) { return r * 180.0 / M_PI; }

    /* ZYX 欧拉角 → 四元数 (Eigen 内置) */
    static Quat eulerZYXToQuat(const Vec3& eDeg)
    {
        return Eigen::AngleAxisd(deg2rad(eDeg.z()), Vec3::UnitZ()) *
               Eigen::AngleAxisd(deg2rad(eDeg.y()), Vec3::UnitY()) *
               Eigen::AngleAxisd(deg2rad(eDeg.x()), Vec3::UnitX());
    }

    /* 四元数 → ZYX 欧拉角 (rad) */
    static Vec3 quatToEulerZYX(const Quat& q)
    {
        const auto& w = q.w();
        const auto& x = q.x();
        const auto& y = q.y();
        const auto& z = q.z();

        double roll  = std::atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
        double sp    = 2 * (w * y - z * x);
        double pitch = std::asin(std::clamp(sp, -1.0, 1.0));
        double yaw   = std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
        return Vec3(roll, pitch, yaw);
    }

    /* 误差四元数 qErr = qTarget * qCurrent.conjugate() */
    static Quat quatError(const Quat& qTarget, const Quat& qCurrent)
    {
        return qTarget * qCurrent.conjugate();
    }

    /* X/Y 双环 PID，返回 3 轴力矩指令（Z 分量后面会被滑模覆盖） */
    Vec3 computeControlBeforeBalancing(const Quat& qCurrent, const Vec3& angleCurrentDeg, const Vec3& wBodyDeg);

public:
    AttitudePDController(GyroScope& gyro, Fan& fan)
        : gyro_(gyro), fan_(fan) {}

    /* 主入口：传入目标欧拉角（ZYX，单位 度） */
    void setAttitudeInBalancing(const Vec3& eulerAngleDeg);
};

#endif //FAN_CONTROL_CODE_ADAPT_C_ATTITUDE_PD_CONTROLLER_H