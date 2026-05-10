//
// Created by msmlab on 2025/11/14.
//

#include "MQTT_server.h"
#include "config_manager.h"

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
	coop_dock_ = new CooperationDock;

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

	coop_dock_->head.head = 0x1D97;    // 表9：帧头 0x1D 0x97
	coop_dock_->tail.checksum = 0x00; // 先置0，后续 memcpy 会覆盖

    const auto& cfg = ConfigManager::getInstance().getConfig();
    SERVER_ADDRESS = cfg.mqtt_server_address;
    CLIENT_ID = cfg.mqtt_client_id;
    QOS = cfg.mqtt_qos;
    plane_data_topic = "attitude/data";
    cmd_plane_basic_topic = "attitude/basic";
    cmd_plane_trajectory_topic = "attitude/trajectory";
    cmd_plane_power_topic = "attitude/power";
    fan_torque_topic   = "attitude/fan_torque";
    fan_velocity_topic = "attitude/fan_velocity";
    wheel_test_topic = "attitude/wheel";
    balance_topic = "attitude/balance";
    fan_calibration_topic = "attitude/calibration";
	coop_dock_topic = "attitude/cooperation";

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

    flag_balance_ = false;
    if_need_balancing_ = false;

    running_ = false;

    if_open_ = false;

    if_need_fan_calibration_ = false;
    flag_fan_calibration_ = false;

    if_receive_attitude_basic_ = false;

    if_receive_fan_torque_ = false;
    if_receive_fan_velocity_ = false;
    fan_torque_data_ = TorqueData{};
    fan_vel_data_ = VelData{};


    if_power_off_ = false;
	if_receive_coop_dock_ = false;

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
 	delete coop_dock_;
}

void CallBack::message_arrived(mqtt::const_message_ptr msg)
{
    const bool is_retained = msg->is_retained();
    if (msg->get_topic() == "attitude/activate")
    {
        std::cout << "On topic: " << msg->get_topic() << std::endl;
        if (!is_retained) {
            task_command_received_.store(true, std::memory_order_release);
        }
        return;
    }
    if (msg->get_topic() == "attitude/basic")
    {
        std::cout << "On topic: " << msg->get_topic() << std::endl;

        if_power_off_ = false;
        std::cout << "------------- 指令开机 -----------" << if_power_off_ << std::endl;

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
        //cmd_basic_ = (CmdBasic*)&buffer;
        std::memcpy(cmd_basic_, buffer, idx);
        cmd_basic_->data.pos_x /= 100.0;
        cmd_basic_->data.pos_y /= 100.0;
        cmd_basic_->data.rot_z /= 100.0;
        cmd_basic_->data.yaw /= 100.0;
        cmd_basic_->data.pitch /= 100.0;
        cmd_basic_->data.roll /= 100.0;

        std::cout << "----- CmdBasic arrived -----\n"
            << " device_id: " << std::hex << std::setfill('0') << std::setw(2) << (int)cmd_basic_->data.device_id
            << " cmd_type: " << std::hex << std::setfill('0') << std::setw(2) << (int)cmd_basic_->data.cmd_type
            << " pos_x: " << std::dec << cmd_basic_->data.pos_x
            << " pos_y: " << std::dec << cmd_basic_->data.pos_y
            << " rot_z: " << std::dec << cmd_basic_->data.rot_z
            << " yaw: " << std::dec << (int)cmd_basic_->data.yaw
            << " pitch: " << std::dec << (int)cmd_basic_->data.pitch
            << " roll: " << std::dec << (int)cmd_basic_->data.roll << "\n";

        attitude_data_.roll = cmd_basic_->data.roll;
        attitude_data_.pitch = cmd_basic_->data.pitch;
        attitude_data_.yaw = cmd_basic_->data.yaw;
        flag_balance_ = true;

        if_receive_attitude_basic_ = true;

        // 进入姿态闭环时，退出风扇两种模式
        if_receive_fan_torque_ = false;
        if_receive_fan_velocity_ = false;
        if_receive_coop_dock_ = false; // 退出对接模式

        if_power_off_ = false;
        if (!is_retained) {
            task_command_received_.store(true, std::memory_order_release);
        }
    }
    else if (msg->get_topic() == "attitude/trajectory")
    {
        std::cout << "On topic: " << msg->get_topic() << std::endl;

        if_power_off_ = false;
        std::cout << "------------- 指令开机 -----------" << if_power_off_ << std::endl;

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

        std::cout << "----- CmdPlaneTrajectory arrived -----\n"
            << " device_id: " << std::hex << std::setfill('0') << std::setw(2) << (int)cmd_trajectory_->data.device_id
            << " cmd_type: " << std::hex << std::setfill('0') << std::setw(2) << (int)cmd_trajectory_->data.cmd_type
            << " traj_id: " << std::hex << std::setfill('0') << std::setw(2) << (int)cmd_trajectory_->data.traj_id
            << " start: " << std::hex << std::setfill('0') << std::setw(2) << (int)cmd_trajectory_->data.start << "\n";
        // delete d;
        if (!is_retained && cmd_trajectory_->data.start) {
            task_command_received_.store(true, std::memory_order_release);
        }
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
        if(cmd_power_->data.cmd_data == 0)
        {
            if_power_off_ = true;
            if_receive_fan_torque_ = false;
            if_receive_fan_velocity_ = false;
            if_receive_attitude_basic_ = false;
            if_need_balancing_ = false;
            if_receive_coop_dock_ = false;

            std::cout << "------------- 指令关机 -----------" << std::endl;
        }
        else
        {
            if_power_off_ = false;
            if (!is_retained) {
                task_command_received_.store(true, std::memory_order_release);
            }
        }

        std::cout << "----- CmdPower arrived -----\n"
            << " device_id: " << std::hex << std::setfill('0') << std::setw(2) << (int)cmd_power_->data.device_id
            << " cmd_type: " << std::hex << std::setfill('0') << std::setw(2) << (int)cmd_power_->data.cmd_type
            << " cmd_data: " << std::hex << std::setfill('0') << std::setw(4) << (int)cmd_power_->data.cmd_data << "\n";
    }
    else if (msg->get_topic() == "attitude/fan_torque")
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

        std::cout << "----- FanTest arrived -----\n"
            << " device_id: " << std::hex << std::setfill('0') << std::setw(2) << (int)fan_test_->data.device_id
            << " cmd_type: " << std::hex << std::setfill('0') << std::setw(2) << (int)fan_test_->data.cmd_type
            << " cmd_data: " << std::hex << std::setfill('0') << std::setw(2) << (int)fan_test_->data.fan_dir
            << " torque_x: " << (int)fan_test_->data.torque_x / 100
            << " torque_y: " << (int)fan_test_->data.torque_y / 100
            << " torque_z: " << (int)fan_test_->data.torque_z / 100 << "\n";
         const float sx = (fan_test_->data.fan_dir & 0x04) ? -1.0f : 1.0f; // bit2 -> x
         const float sy = (fan_test_->data.fan_dir & 0x02) ? -1.0f : 1.0f; // bit1 -> y
         const float sz = (fan_test_->data.fan_dir & 0x01) ? -1.0f : 1.0f; // bit0 -> z

         fan_torque_data_.tx = sx * (static_cast<float>(fan_test_->data.torque_x) / 100.0f);
         fan_torque_data_.ty = sy * (static_cast<float>(fan_test_->data.torque_y) / 100.0f);
         fan_torque_data_.tz = sz * (static_cast<float>(fan_test_->data.torque_z) / 100.0f);

        // 触发纯力矩控制模式：需要退出其他模式（闭环姿态控制 / 自动调平 / 对接）
         if_receive_fan_torque_ = true;
         if_receive_fan_velocity_ = false;
         if_receive_attitude_basic_ = false;
         if_need_balancing_ = false;
         if_receive_coop_dock_ = false;

         if_power_off_ = false;
         if (!is_retained) {
             task_command_received_.store(true, std::memory_order_release);
         }
    }
    else if (msg->get_topic() == "attitude/fan_velocity")
    {
        std::cout << "On topic: " << msg->get_topic() << std::endl;
        const std::string& pl = msg->get_payload();
        uint8_t buffer[200] = {};
        int idx = 0;
        convert_msg(pl, buffer, idx);

        // 这里我复用 FanTest 结构：fan_dir + 三个字节
        // 约定：torque_x/y/z 字段在该 topic 下表示 wx/wy/wz（×100，单位 deg/s）
        if (idx != sizeof(FanTest))
        {
            std::cerr << "[WARN] payload size " << idx
                      << " != " << sizeof(FanTest) << " bytes, drop\n";
            return;
        }
        std::memcpy(fan_test_, buffer, idx);

        const float sx = (fan_test_->data.fan_dir & 0x04) ? -1.0f : 1.0f;
        const float sy = (fan_test_->data.fan_dir & 0x02) ? -1.0f : 1.0f;
        const float sz = (fan_test_->data.fan_dir & 0x01) ? -1.0f : 1.0f;

        fan_vel_data_.wx = sx * (static_cast<float>(fan_test_->data.torque_x) / 100.0f);
        fan_vel_data_.wy = sy * (static_cast<float>(fan_test_->data.torque_y) / 100.0f);
        fan_vel_data_.wz = sz * (static_cast<float>(fan_test_->data.torque_z) / 100.0f);

        // 模式互斥：进入速度模式，退出调平/姿态/力矩模式/对接
        if_receive_fan_velocity_ = true;
        if_receive_fan_torque_ = false;
        if_receive_attitude_basic_ = false;
        if_need_balancing_ = false;
        if_receive_coop_dock_ = false;

        if_power_off_ = false;
        if (!is_retained) {
            task_command_received_.store(true, std::memory_order_release);
        }
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

        std::cout << "----- WheelTest arrived -----\n"
            << " device_id: " << std::hex << std::setfill('0') << std::setw(4) << (int)wheel_test_->data.device_id
            << " cmd_type: " << std::hex << std::setfill('0') << std::setw(4) << (int)wheel_test_->data.cmd_type
            << " wheel_model: " << std::hex << std::setfill('0') << std::setw(4) << (int)wheel_test_->data.wheel_model
            << " wheel_dir: " << std::hex << std::setfill('0') << std::setw(4) << (int)wheel_test_->data.wheel_dir
            << " wheel_current: " << (int)wheel_test_->data.wheel_current / 100
            << " wheel_rpm: " << std::hex << std::setfill('0') << std::setw(4) << (int)wheel_test_->data.wheel_rpm
            << "\n";
        if_power_off_ = false;
        if (!is_retained) {
            task_command_received_.store(true, std::memory_order_release);
        }
    }
    else if (msg->get_topic() == "attitude/balance")
    {
        std::cout << "On topic: " << msg->get_topic() << std::endl;
        const std::string& pl = msg->get_payload();
        uint8_t buffer[200] = {};
        int idx = 0;
        convert_msg(pl, buffer, idx);

        if_power_off_ = false;
        std::cout << "------------- 指令开机 -----------" << if_power_off_ << std::endl;

        if (idx != sizeof(Balance))
        {
            std::cerr << "[WARN] payload size " << idx
                << " != " << sizeof(Balance) << " bytes, drop\n";
            return;
        }
        std::memcpy(balance_, buffer, idx);

        std::cout << "----- BalanceData arrived -----\n"
            << " len: " << std::hex << std::setfill('0') << std::setw(2) << (int)balance_->data.len
            << " cmd: " << std::hex << std::setfill('0') << std::setw(2) << (int)balance_->data.cmd
            << " device_id: " << std::hex << std::setfill('0') << std::setw(2) << (int)balance_->data.device_id
            << " balance_set: " << std::hex << std::setfill('0') << std::setw(2) << (int)balance_->data.balance_set
            << " balance_reset: " << std::hex << std::setfill('0') << std::setw(2) << (int)balance_->data.balance_reset
            << "\n";
        if (balance_->data.balance_reset) {
            flag_balance_ = false;
        }
        if(!flag_balance_) // 没完成过自动调平
        {
            if(balance_->data.balance_set)
                if_need_balancing_ = true;
            else
                if_need_balancing_ = false;
        }
        else
        {
            // 之前完成过调平，需要balance_reset和balance_set同时为true才能重新调平
            if(balance_->data.balance_reset && balance_->data.balance_set)
                if_need_balancing_ = true;
            else if(balance_->data.balance_set && !balance_->data.balance_reset)
            {
                // 只有balance_set，没有balance_reset，不允许重新调平
                // 但为了支持姿态控制后直接调平，我们强制允许
                if_need_balancing_ = true;
                flag_balance_ = false; // 重置标志，允许调平
            }
            else
                if_need_balancing_ = false;
        }

         if_receive_fan_torque_ = false;
         if_receive_coop_dock_ = false;
         if_power_off_ = false;
         if (!is_retained && if_need_balancing_) {
             task_command_received_.store(true, std::memory_order_release);
         }
     }
    else if (msg->get_topic() == "attitude/calibration1")
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

        std::cout << "----- FanCalibration arrived -----\n"
            << " len: " << std::hex << std::setfill('0') << std::setw(2) << (int)fan_calibration_->data.len
            << " cmd: " << std::hex << std::setfill('0') << std::setw(2) << (int)fan_calibration_->data.cmd
            << " device_id: " << std::hex << std::setfill('0') << std::setw(2) << (int)fan_calibration_->data.device_id
            << " fan_set: " << std::hex << std::setfill('0') << std::setw(2) << (int)fan_calibration_->data.fan_set
            << " fan_reset: " << std::hex << std::setfill('0') << std::setw(2) << (int)fan_calibration_->data.fan_reset
            << "\n";

        if(!flag_fan_calibration_)
        {
            if(fan_calibration_->data.fan_set)
                if_need_fan_calibration_ = true;
            else
                if_need_fan_calibration_ = false;
        }
        else
        {
            if(fan_calibration_->data.fan_set && fan_calibration_->data.fan_reset)
                if_need_fan_calibration_ = true;
            else
                if_need_fan_calibration_ = false;
        }
        if (!is_retained && if_need_fan_calibration_) {
            task_command_received_.store(true, std::memory_order_release);
        }
    }
 	else if (msg->get_topic() == "attitude/calibration")
    {
        std::cout << "On topic: " << msg->get_topic() << std::endl;

        // 你现有逻辑：收到指令一般认为进入工作状态
        if_power_off_ = false;
        std::cout << "------------- 指令开机 -----------" << if_power_off_ << std::endl;

        const std::string& pl = msg->get_payload();
        uint8_t buffer[200] = {};
        int idx = 0;
        convert_msg(pl, buffer, idx);

        // 表9：6字节
        if (idx != sizeof(CooperationDock))
        {
            std::cerr << "[WARN] payload size " << idx
                      << " != " << sizeof(CooperationDock) << " bytes, drop\n";
            return;
        }

        // XOR 校验：前5字节异或 == 第6字节
        uint8_t x = 0x00;
        for (int i = 0; i < idx - 1; ++i) x ^= buffer[i];
        if (x != buffer[idx - 1])
        {
            std::cerr << "[WARN] CooperationDock xor mismatch: calc=0x"
                      << std::hex << std::setfill('0') << std::setw(2) << (int)x
                      << " recv=0x" << std::setw(2) << (int)buffer[idx - 1]
                      << " drop\n";
            return;
        }

        std::memcpy(coop_dock_, buffer, idx);

 	    coop_dock_data_ = coop_dock_->data;

        std::cout << "----- CooperationDock arrived -----\n"
                  << " self_device_id: " << std::hex << std::setfill('0') << std::setw(2)
                  << (int)coop_dock_->data.self_device_id
                  << " cmd_type: " << std::hex << std::setfill('0') << std::setw(2)
                  << (int)coop_dock_->data.cmd_type
                  << " dock_device_id: " << std::hex << std::setfill('0') << std::setw(2)
                  << (int)coop_dock_->data.dock_device_id
                  << " checksum: " << std::hex << std::setfill('0') << std::setw(2)
                  << (int)coop_dock_->tail.checksum
                  << "\n";

        // 可选：强约束 cmd_type == 0x17
        if (coop_dock_->data.cmd_type != 0x17)
        {
            std::cerr << "[WARN] CooperationDock cmd_type != 0x17, got=0x"
                      << std::hex << std::setfill('0') << std::setw(2)
                      << (int)coop_dock_->data.cmd_type << "\n";
        }

        if_receive_coop_dock_ = true;
        if_receive_attitude_basic_ = false;
        if_receive_fan_torque_ = false;
        if_receive_fan_velocity_ = false;
        if_need_balancing_ = false;
        if (!is_retained) {
            task_command_received_.store(true, std::memory_order_release);
        }
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

void CallBack::setFlagBalance(bool flag)
{
    if(flag)
    {
        flag_balance_ = true;
        if_need_balancing_ = false;
    }
    else
        flag_balance_ = false;
}
