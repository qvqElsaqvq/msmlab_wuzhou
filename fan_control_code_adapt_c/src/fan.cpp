//
// Created by msmlab on 2025/11/14.
//

#include "fan.h"

void Fan::sendTorque(float tx, float ty, float tz) {
    uint8_t dir = 0;
    if (tx < 0) dir |= 1 << 2;
    if (ty < 0) dir |= 1 << 1;
    if (tz < 0) dir |= 1 << 0;

    auto toUint8_100 = [](float v) {
        int iv = static_cast<int>(std::fabs(v) * 100.0f);
        return static_cast<uint8_t>(std::min(255, std::max(0, iv)));
    };

    FanControl fan_control{
        .device_id = 0x00,
        .direction = dir,
        // .torque_x = 10,
        // .torque_y = 0,
        // .torque_z = 0,
        .torque_x = toUint8_100(tx),
        .torque_y = toUint8_100(ty),
        .torque_z = toUint8_100(tz),
    };
    ser_.write(0x01, fan_control);
    // std::cout << "[Fan] Sending direction=" << std::hex << std::setfill('0') << std::setw(2) << (int)dir
    // << ", torque_x=" << std::hex << std::setfill('0') << std::setw(2) << (int)fan_control.torque_x
    // << ", torque_y=" << std::hex << std::setfill('0') << std::setw(2) << (int)fan_control.torque_y
    // << ", torque_z=" << std::hex << std::setfill('0') << std::setw(2) << (int)fan_control.torque_z << std::endl;
    // << ", torque_x=" << std::dec << (int)fan_control.torque_x
    // << ", torque_y=" << std::dec << (int)fan_control.torque_y
    // << ", torque_z=" << std::dec << (int)fan_control.torque_z << std::endl;
}

Fan::Fan(msmserial::MsMSerial& serial): ser_(serial)
{
    std::cout << "Fan::Fan init" << std::endl;

    ser_.registerCallback(0x08, [this](const FanCalibrationData& msg)
    {
        // std::cout << "[Fan receive] device_id=" << (int)msg.device_id << ", fan_flag=" << (int)msg.fan_flag <<
        //     ", fan_set=" << (int)msg.fan_set << std::endl;
    });
}
