//
// Created by msmlab on 2025/11/14.
//

#include "leadscrew_controller.h"

LeadScrewController::LeadScrewController(msmserial::MsMSerial& serial) : ser_(serial)
{
    std::cout << "[LeadScrewController] init" << std::endl;

    if_power_off_ = false;

    ser_.registerCallback(0x09, [this](const LeadScrewAlarm& msg)
    {
        // std::cout << "[LeadScrewController receive] device_id=" << (int)msg.device_id << ", alarm_x=" << (int)msg.alarm_x <<
        //     ", alarm_y=" << (int)msg.alarm_y << ", alarm_z=" << (int)msg.alarm_z << std::endl;
    });
}


void LeadScrewController::moveTo(const std::vector<int16_t>& location)
{
    if (!if_power_off_)
    {
        std::cout << ">>>>>>>>>>>location size: " << location.size() << std::endl;
        last_step_.assign({0, 0, 0});
        LeadScrewControl leadscrew_control{
            .device_id = 0x3A,
            .dist_x = 0,
            .dist_y = 0,
            .dist_z = 0,
        };
        if (location.size() == 2) // X/Y
        {
            leadscrew_control.dist_x = location[0];
            leadscrew_control.dist_y = location[1];
            ser_.write(0x04, leadscrew_control);
            // std::cout << "[LeadScrewController] moving XY: " << std::endl;
            // std::cout << "x=" << std::dec << leadscrew_control.dist_x
            // << ", y=" << std::dec << leadscrew_control.dist_y << std::endl;
            last_step_[0] = location[0];
            last_step_[1] = location[1];
        }
        else if (location.size() == 1) // Z
        {
            leadscrew_control.dist_z = location[0];
            ser_.write(0x04, leadscrew_control);
            // std::cout << "[LeadScrewController] moving Z: " << std::endl;
            // std::cout << "z=" << std::dec << leadscrew_control.dist_z << std::endl;
            last_step_[2] = location[0];
        }
    }
}

const std::vector<int16_t>& LeadScrewController::getCurrentPositions() const
{
    return last_step_;
}

void LeadScrewController::setIfPowerOff(bool if_power_off)
{
    if_power_off_ = if_power_off;
}
