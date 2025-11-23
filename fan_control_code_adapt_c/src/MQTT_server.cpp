//
// Created by msmlab on 2025/11/14.
//

#include "MQTT_server.h"

CallBack::CallBack()
{
    plane_ = new Plane;
    cmd_basic_ = new CmdBasic;
    cmd_power_ = new CmdPower;
    fan_test_ = new FanTest;
    wheel_test_ = new WheelTest;
    balance_ = new Balance;
    fan_calibration_ = new FanCalibration;
    cmd_trajectory_ = new CmdTrajectory;

    plane_->head.head = 0x4D47;
    plane_->tail.checksum = 0x00;

    cmd_basic_->head.head = 0x1D97;
    cmd_basic_->tail.checksum = 0x00;

    cmd_trajectory_->head.head = 0x1D97;
    cmd_trajectory_->tail.checksum = 0x00;

    cmd_power_->head.head = 0x1D97;
    cmd_power_->tail.checksum = 0x00;

    fan_test_->head.head = 0x1D97;
    fan_test_->tail.checksum = 0x00;

    wheel_test_->head.head = 0x1D97;
    wheel_test_->tail.checksum = 0x00;

    balance_->head.head = 0x5A47;
    balance_->tail.checksum = 0x00;

    fan_calibration_->head.head = 0x5A47;
    fan_calibration_->tail.checksum = 0x00;

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
    if_open_ = false;
}

CallBack::~CallBack()
{
    delete plane_;
    delete cmd_basic_;
    delete cmd_power_;
    delete fan_test_;
    delete wheel_test_;
    delete balance_;
    delete fan_calibration_;
}

void CallBack::message_arrived(mqtt::const_message_ptr msg)
{
    if (msg->get_topic() == "attitude/basic")
    {
        std::cout << "On topic: " << msg->get_topic() << std::endl;
        const std::string& pl = msg->get_payload();
        uint8_t buffer[200] = {};
        int idx = 0;
        convert_msg(pl, buffer, idx);

        if (idx != sizeof(CmdBasic))
        {
            std::cerr << "[WARN] payload size " << idx
                << " != " << sizeof(CmdBasic) << " bytes, drop\n";
            return;
        }
        std::memcpy(cmd_basic_, buffer, idx);

        // std::cout << "----- CmdBasic arrived -----\n"
        //     << " device_id: " << std::hex << std::setfill('0') << std::setw(2) << (int)cmd_basic_->data.device_id
        //     << " cmd_type: " << std::hex << std::setfill('0') << std::setw(2) << (int)cmd_basic_->data.cmd_type
        //     << " pos_x: " << cmd_basic_->data.pos_x / 100
        //     << " pos_y: " << cmd_basic_->data.pos_y / 100
        //     << " rot_z: " << cmd_basic_->data.rot_z / 100
        //     << " yaw: " << (int)cmd_basic_->data.yaw / 100
        //     << " pitch: " << (int)cmd_basic_->data.pitch / 100
        //     << " roll: " << (int)cmd_basic_->data.roll / 100 << "\n";
    }
    else if (msg->get_topic() == "attitude/trajectory")
    {
        std::cout << "On topic: " << msg->get_topic() << std::endl;
        const std::string& pl = msg->get_payload();
        uint8_t buffer[200] = {};
        int idx = 0;
        convert_msg(pl, buffer, idx);

        if (idx != sizeof(CmdTrajectory))
        {
            std::cerr << "[WARN] payload size " << idx
                << " != " << sizeof(CmdTrajectory) << " bytes, drop\n";
            return;
        }

        // CmdTrajectory* d = new CmdTrajectory;
        // d = (CmdTrajectory*)&buffer[0];
        std::memcpy(cmd_trajectory_, buffer, idx);

        // std::cout << "----- CmdPlaneTrajectory arrived -----\n"
        //     << " device_id: " << std::hex << std::setfill('0') << std::setw(2) << (int)cmd_trajectory_->data.device_id
        //     << " cmd_type: " << std::hex << std::setfill('0') << std::setw(2) << (int)cmd_trajectory_->data.cmd_type
        //     << " traj_id: " << std::hex << std::setfill('0') << std::setw(2) << (int)cmd_trajectory_->data.traj_id
        //     << " start: " << std::hex << std::setfill('0') << std::setw(2) << (int)cmd_trajectory_->data.start << "\n";
        // delete d;
    }
    else if (msg->get_topic() == "attitude/power")
    {
        std::cout << "On topic: " << msg->get_topic() << std::endl;
        const std::string& pl = msg->get_payload();
        uint8_t buffer[200] = {};
        int idx = 0;
        convert_msg(pl, buffer, idx);

        if (idx != sizeof(CmdPower))
        {
            std::cerr << "[WARN] payload size " << idx
                << " != " << sizeof(CmdPower) << " bytes, drop\n";
            return;
        }
        std::memcpy(cmd_power_, buffer, idx);
        // std::cout << "----- CmdPower arrived -----\n"
        //     << " device_id: " << std::hex << std::setfill('0') << std::setw(2) << (int)cmd_power_->data.device_id
        //     << " cmd_type: " << std::hex << std::setfill('0') << std::setw(2) << (int)cmd_power_->data.cmd_type
        //     << " cmd_data: " << std::hex << std::setfill('0') << std::setw(2) << (int)cmd_power_->data.cmd_data << "\n";
    }
    else if (msg->get_topic() == "attitude/fan")
    {
        std::cout << "On topic: " << msg->get_topic() << std::endl;
        const std::string& pl = msg->get_payload();
        uint8_t buffer[200] = {};
        int idx = 0;
        convert_msg(pl, buffer, idx);

        if (idx != sizeof(FanTest))
        {
            std::cerr << "[WARN] payload size " << idx
                << " != " << sizeof(FanTest) << " bytes, drop\n";
            return;
        }
        std::memcpy(fan_test_, buffer, idx);

        // std::cout << "----- FanTest arrived -----\n"
        //     << " device_id: " << std::hex << std::setfill('0') << std::setw(2) << (int)fan_test_->data.device_id
        //     << " cmd_type: " << std::hex << std::setfill('0') << std::setw(2) << (int)fan_test_->data.cmd_type
        //     << " cmd_data: " << std::hex << std::setfill('0') << std::setw(2) << (int)fan_test_->data.fan_dir
        //     << " torque_x: " << (int)fan_test_->data.torque_x / 100
        //     << " torque_y: " << (int)fan_test_->data.torque_y / 100
        //     << " torque_z: " << (int)fan_test_->data.torque_z / 100 << "\n";
    }
    else if (msg->get_topic() == "attitude/wheel")
    {
        std::cout << "On topic: " << msg->get_topic() << std::endl;
        const std::string& pl = msg->get_payload();
        uint8_t buffer[200] = {};
        int idx = 0;
        convert_msg(pl, buffer, idx);

        if (idx != sizeof(WheelTest))
        {
            std::cerr << "[WARN] payload size " << idx
                << " != " << sizeof(WheelTest) << " bytes, drop\n";
            return;
        }
        std::memcpy(wheel_test_, buffer, idx);

        // std::cout << "----- WheelTest arrived -----\n"
        //     << " device_id: " << std::hex << std::setfill('0') << std::setw(4) << (int)wheel_test_->data.device_id
        //     << " cmd_type: " << std::hex << std::setfill('0') << std::setw(4) << (int)wheel_test_->data.cmd_type
        //     << " wheel_model: " << std::hex << std::setfill('0') << std::setw(4) << (int)wheel_test_->data.wheel_model
        //     << " wheel_dir: " << std::hex << std::setfill('0') << std::setw(4) << (int)wheel_test_->data.wheel_dir
        //     << " wheel_current: " << (int)wheel_test_->data.wheel_current / 100
        //     << " wheel_rpm: " << std::hex << std::setfill('0') << std::setw(4) << (int)wheel_test_->data.wheel_rpm
        //     << "\n";
    }
    else if (msg->get_topic() == "attitude/balance")
    {
        std::cout << "On topic: " << msg->get_topic() << std::endl;
        const std::string& pl = msg->get_payload();
        uint8_t buffer[200] = {};
        int idx = 0;
        convert_msg(pl, buffer, idx);

        if (idx != sizeof(BalanceData))
        {
            std::cerr << "[WARN] payload size " << idx
                << " != " << sizeof(BalanceData) << " bytes, drop\n";
            return;
        }
        std::memcpy(balance_, buffer, idx);

        // std::cout << "----- BalanceData arrived -----\n"
        //     << " len: " << std::hex << std::setfill('0') << std::setw(2) << (int)balance_->data.len
        //     << " cmd: " << std::hex << std::setfill('0') << std::setw(2) << (int)balance_->data.cmd
        //     << " device_id: " << std::hex << std::setfill('0') << std::setw(2) << (int)balance_->data.device_id
        //     << " balance_set: " << std::hex << std::setfill('0') << std::setw(2) << (int)balance_->data.balance_set
        //     << " balance_reset: " << std::hex << std::setfill('0') << std::setw(2) << (int)balance_->data.balance_reset
        //     << "\n";
    }
    else if (msg->get_topic() == "attitude/calibration")
    {
        std::cout << "On topic: " << msg->get_topic() << std::endl;
        const std::string& pl = msg->get_payload();
        uint8_t buffer[200] = {};
        int idx = 0;
        convert_msg(pl, buffer, idx);

        if (idx != sizeof(FanCalibration))
        {
            std::cerr << "[WARN] payload size " << idx
                << " != " << sizeof(FanCalibration) << " bytes, drop\n";
            return;
        }
        std::memcpy(fan_calibration_, buffer, idx);

        // std::cout << "----- FanCalibration arrived -----\n"
        //     << " len: " << std::hex << std::setfill('0') << std::setw(2) << (int)fan_calibration_->data.len
        //     << " cmd: " << std::hex << std::setfill('0') << std::setw(2) << (int)fan_calibration_->data.cmd
        //     << " device_id: " << std::hex << std::setfill('0') << std::setw(2) << (int)fan_calibration_->data.device_id
        //     << " fan_set: " << std::hex << std::setfill('0') << std::setw(2) << (int)fan_calibration_->data.fan_set
        //     << " fan_reset: " << std::hex << std::setfill('0') << std::setw(2) << (int)fan_calibration_->data.fan_reset
        //     << "\n";
    }
}

void CallBack::convert_msg(const std::string& pl, uint8_t* buffer, int& idx)
{
    std::stringstream ss(pl);
    // std::cout << "buffer[idx]: " << std::endl;
    while (!ss.eof())
    {
        int temp;
        ss >> std::hex >> temp;
        buffer[idx] = temp;
        // std::cout << std::hex << std::setfill('0') << std::setw(2) << temp << " ";
        idx += 1;
    }
    // std::cout << " pl: " << pl << std::endl;
}
