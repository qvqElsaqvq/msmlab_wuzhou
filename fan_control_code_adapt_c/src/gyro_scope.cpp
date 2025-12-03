//
// Created by msmlab on 2025/11/14.
//

#include "gyro_scope.h"

void GyroScope::setAngularVelocity(double wx, double wy, double wz)
{
    latestAngularVel_.x = wx;
    latestAngularVel_.y = wy;
    latestAngularVel_.z = wz;
}

void GyroScope::setAttitude(double roll, double pitch, double yaw)
{
    latestAttitude_.x = roll;
    latestAttitude_.y = pitch;
    latestAttitude_.z = yaw;
}

GyroScope::GyroScope(msmserial::MsMSerial& msm_serial): ser_(msm_serial)
{
    std::cout << "[GyroScope] init" << std::endl;

    ser_.registerCallback(0x05, [this](const GyroScopeData& msg)
    {
        setAngularVelocity(msg.wx / 100.0, msg.wy / 100.0, msg.wz / 100.0);
        setAttitude(msg.roll / 100.0, msg.pitch / 100.0, msg.yaw / 100.0);

        // std::cout << "[GyroScope receive] wx=" << msg.wx / 100.0
        // << ", wy=" << msg.wy / 100.0
        // << ", wz=" << msg.wz / 100.0 << std::endl;
        std::cout << "[GyroScope receive] roll=" << msg.roll / 100.0
        << ", pitch=" << msg.pitch / 100.0
        << ", yaw=" << msg.yaw / 100.0 << std::endl;
    });
}
