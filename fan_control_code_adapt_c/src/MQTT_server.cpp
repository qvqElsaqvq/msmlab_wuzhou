//
// Created by msmlab on 2025/11/14.
//

#include "MQTT_server.h"

INIReader ini("satellite.ini");

CallBack::CallBack()
{
    plane_.head.head = 0x4D47;
    plane_.tail.checksum = 0x00;

    cmd_basic_.head.head = 0x1D97;
    cmd_basic_.tail.checksum = 0x00;

    cmd_trajectory_.head.head = 0x1D97;
    cmd_trajectory_.tail.checksum = 0x00;

    cmd_power_.head.head = 0x1D97;
    cmd_power_.tail.checksum = 0x00;

    SERVER_ADDRESS = "mqtt://broker.emqx.io:1883";
    CLIENT_ID = "satellite_client";
    QOS = 1;
    plane_data_topic = "satellite/data";
    cmd_plane_basic_topic = "satellite/basic";
    cmd_plane_trajectory_topic = "satellite/trajectory";
    cmd_plane_power_topic = "satellite/power";

    wx_ = 0.0;
    wy_ = 0.0;
    wz_ = 0.0;
    roll_ = 0.0;
    pitch_ = 0.0;
    yaw_ = 0.0;
    q0_ = 0.0;
    q1_ = 0.0;
    q2_ = 0.0;
    q3_ = 0.0;
    QOS = 1;

    flag_balance_ = false;
    flag_attitude_euler_ = false;
    flag_attitude_quat_ = false;
    running_ = false;
}

void CallBack::send_plane_data()
{
    plane_.data.device_id = 0x01;
    plane_.data.platform_type = 0xF1;
    plane_.data.cmd_count = 0x01;
    plane_.data.file_count = 0x01;
    plane_.data.platform_status = 0x00;
    plane_.data.wx = 50;
    plane_.data.wy = 50;
    plane_.data.wz = 50;
    plane_.data.roll = 30;
    plane_.data.pitch = 30;
    plane_.data.yaw = 30;
    plane_.data.gyro_fault = 0x00;
    plane_.data.wheel_dir = 0x55;
    plane_.data.wheel_current = 100;
    plane_.data.wheel_rpm = 3000;
    plane_.data.wheel_fault = 0x00;
    plane_.data.payload_mass = 100;
    plane_.data.pwr_v1 = 50;
    plane_.data.pwr_v2 = 50;
    plane_.data.pwr_v3 = 50;
    plane_.data.pwr_v4 = 50;
    plane_.data.pwr_i1 = 20;
    plane_.data.pwr_i2 = 20;
    plane_.data.pwr_i3 = 20;
    plane_.data.pwr_i4 = 20;
    plane_.data.battery_percent = 80;
    plane_.data.traj_ready = 0x01;
    plane_.data.thrust_x = 200;
    plane_.data.thrust_y = 200;
    plane_.data.thrust_z = 200;
    plane_.data.torque_roll = 300;
    plane_.data.torque_pitch = 300;
    plane_.data.torque_yaw = 300;
    plane_.data.balance_flag = 0x00;
    plane_.data.balance_set = 0x01;
    plane_.data.fan_calibration_flag = 0x00;
    plane_.data.fan_calibration_set = 0x01;
    for(int i = 0; i < 7; i++)
    {
        plane_.data.reserved[i] = 0x00;
    }
}
