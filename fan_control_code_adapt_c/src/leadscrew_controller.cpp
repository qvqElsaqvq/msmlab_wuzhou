//
// Created by msmlab on 2025/11/14.
//

#include "leadscrew_controller.h"

void LeadScrewController::moveTo(const std::vector<float>& location)
{
    LeadScrewControl leadscrew_control{
        .device_id = 0x00,
        .dist_x = 0.0,
        .dist_y = 0.0,
        .dist_z = 0.0,
    };
    if (location.size() == 2)  // X/Y
    {
        leadscrew_control.dist_x = location[0];
        leadscrew_control.dist_y = location[1];
    }
    else if (location.size() == 1)  // Z
    {
        leadscrew_control.dist_z = location[0];
    }
    ser_.write(0x03, leadscrew_control);
}