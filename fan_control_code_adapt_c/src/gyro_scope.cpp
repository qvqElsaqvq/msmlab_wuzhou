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
    ser_.registerCallback(0x04, [this](const GyroScopeData& msg)
    {
        setAngularVelocity(msg.wx, msg.wy, msg.wz);
        setAttitude(msg.roll, msg.pitch, msg.yaw);

        std::cout << "[GyroScope receive] wx=" << msg.wx << ", wy=" << msg.wy << ", wz=" << msg.wz << std::endl;
        std::cout << "[GyroScope receive] roll=" << msg.roll << ", pitch=" << msg.pitch << ", yaw=" << msg.yaw << std::endl;
    });
}
