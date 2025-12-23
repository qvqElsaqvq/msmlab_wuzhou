//
// Created by msmlab on 2025/11/14.
//

#include "wheel.h"

Wheel::Wheel(msmserial::MsMSerial& serial): ser_(serial)
{
    std::cout << "[Wheel] init" << std::endl;

    if_power_off_ = false;

    ser_.registerCallback(0x06, [this](const WheelData& msg)
    {
        setStauts(msg.device_id, msg.direction, msg.speed);
        // std::cout << "[Wheel] receive id,direction,speed: " << (int) msg.device_id << " " << (int) msg.direction << " "
        //         << (int) msg.speed << std::endl;
    });
}

void Wheel::setStauts(uint8_t wheel_id, uint8_t dir, uint8_t speed)
{
    wheel_status_.id = wheel_id;
    wheel_status_.dir = dir;
    wheel_status_.speed = speed;
}

Wheel::Status Wheel::getStatus()
{
    Status current_status;
    current_status.id = wheel_status_.id;
    current_status.dir = wheel_status_.dir;
    current_status.speed = wheel_status_.speed;
    return current_status;
}

void Wheel::sendFrame(uint8_t wheel_id, uint8_t dir, uint8_t current)
{
    if (!if_power_off_)
    {
        WheelControl wheel_control{
            .device_id = wheel_id,
            .direction = dir,
            .current = current,
        };
        ser_.write(0x02, wheel_control);
        std::cout << "[Wheel] send id=" << wheel_id << ", direction=" << dir << ", current=" << current << std::endl;
    }
}

void Wheel::setIfPowerOff(bool if_power_off)
{
    if_power_off_ = if_power_off;
}
