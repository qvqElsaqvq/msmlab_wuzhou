//
// Created by msmlab on 2025/11/14.
//

#include "MQTT_server.h"

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

    fan_test_.head.head = 0x1D97;
    fan_test_.tail.checksum = 0x00;

    wheel_test_.head.head = 0x1D97;
    wheel_test_.tail.checksum = 0x00;

    balance_.head.head = 0x5A47;
    balance_.tail.checksum = 0x00;

    fan_calibration_.head.head = 0x5A47;
    fan_calibration_.tail.checksum = 0x00;

    SERVER_ADDRESS = "mqtt://112.20.77.50:1883";
    CLIENT_ID = "satellite_client";
    QOS = 1;
    plane_data_topic = "attitude/data";
    cmd_plane_basic_topic = "attitude/basic";
    cmd_plane_trajectory_topic = "attitude/trajectory";
    cmd_plane_power_topic = "attitude/power";
    fan_test_topic = "attitude/fan";
    wheel_test_topic = "attitude/wheel";
    balance_topic = "attitude/balance";
    fan_calibration_topic = "attitude/calibration";

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
