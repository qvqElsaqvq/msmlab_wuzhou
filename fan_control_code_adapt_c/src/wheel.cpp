//
// Created by msmlab on 2025/11/14.
//

#include "wheel.h"

Wheel::Wheel(msmserial::MsMSerial& serial): ser_(serial)
{
    std::cout << "[Wheel] init" << std::endl;

    wheel_pid_.Dout = Vec3(0.0, 0, 0);
    wheel_pid_.Pout = Vec3(0, 0, 0);
    wheel_pid_.Iout = Vec3(0, 0, 0);
    wheel_pid_.last_error = Vec3(0, 0, 0);
    wheel_pid_.max_i_out = Vec3(0, 0, 0);
    wheel_pid_.max_out = Vec3(0, 0, 0);
    wheel_pid_.out = Vec3(0, 0, 0);

    /* PID参数 */
    wheel_pid_.Kp = Vec3(0, 0, 0);
    wheel_pid_.Ki = Vec3(0, 0, 0);
    wheel_pid_.Kd = Vec3(0, 0, 0);

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
    WheelControl wheel_control{
        .device_id = wheel_id,
        .direction = dir,
        .current = current,
    };
    ser_.write(0x02, wheel_control);
    std::cout << "[Wheel] send id=" << wheel_id << ", direction=" << dir << ", current=" << current << std::endl;
}

Wheel::PID Wheel::computePID(PID pid, Vec3& ref, Vec3& set) {
    Vec3 error = set - ref;

    pid.Pout = pid.Kp.array() * error.array();
    pid.Iout.array() += pid.Ki.array() * error.array();
    pid.Iout[0] = std::max(pid.Iout[0], pid.max_i_out[0]);
    pid.Iout[1] = std::max(pid.Iout[1], pid.max_i_out[1]);
    pid.Iout[2] = std::max(pid.Iout[2], pid.max_i_out[2]);
    pid.Dout = pid.Kd.array() * (error.array() - pid.last_error.array());
    pid.out = pid.Pout + pid.Iout + pid.Dout;
    pid.out[0] = std::max(pid.out[0], pid.max_out[0]);
    pid.out[1] = std::max(pid.out[1], pid.max_out[1]);
    pid.out[2] = std::max(pid.out[2], pid.max_out[2]);

    return pid;
}