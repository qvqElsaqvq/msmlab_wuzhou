//
// Created by msmlab on 2025/11/14.
//

#include "MQTT_server.h"

INIReader ini("satellite.ini");

CallBack::CallBack()
{
    BROKER_HOST = "tcp://broker.emqx.io:1883";
    BROKER_PORT = 1883;
    CLIENT_ID = "mqtt_client";
    QOS = 1;
    SUB_TOPIC = "satellite/cmd";
    PUB_TOPIC = "satellite/data";

    if (ini.ParseError() == -1) {
        std::cout << "No config found! Creating default!" << std::endl;
        std::fstream file;
        file.open("config.ini", std::fstream::out);
        file << "[MQTT]" << std::endl;
        file << "BROKER_HOST=" << BROKER_HOST << std::endl;
        file << "BROKER_PORT=" << BROKER_PORT << std::endl;
        file << "CLIENT_ID=" << CLIENT_ID << std::endl;
        file << "SUB_TOPIC=" << SUB_TOPIC << std::endl;
        file << "PUB_TOPIC=" << PUB_TOPIC << std::endl;
        file.close();
    }else if (ini.ParseError() > 0 || ini.ParseError() < -1) {
        std::cout<< "Read config failed!" << std::endl;
        std::cout << ini.ParseErrorMessage() << std::endl;
        return;
    }else {
        std::cout<< "Read config successfully!" << std::endl;
        BROKER_HOST = ini.GetString("MQTT", "BROKER_HOST", BROKER_HOST);
        BROKER_PORT = static_cast<int>(ini.GetInteger("MQTT", "BROKER_PORT", BROKER_PORT));
        CLIENT_ID = ini.GetString("MQTT", "CLIENT_ID", CLIENT_ID);
        QOS = static_cast<int>(ini.GetInteger("MQTT", "QOS", QOS));
        SUB_TOPIC = ini.GetString("MQTT", "SUB_TOPIC", SUB_TOPIC);
        PUB_TOPIC = ini.GetString("MQTT", "PUB_TOPIC", PUB_TOPIC);
    }
}

MQTTServer::MQTTServer(std::string BROKER_HOST, int BROKER_PORT, std::string CLIENT_ID, std::string PUB_TOPIC,
                       std::string SUB_TOPIC):
    client_(BROKER_HOST, CLIENT_ID), broker_host_(BROKER_HOST), broker_port_(BROKER_PORT), client_id_(CLIENT_ID),
    pub_topic_(PUB_TOPIC), sub_topic_(SUB_TOPIC)
{
    client_.set_callback(cb_);
    connOpts_.set_clean_session(true);

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
    flag_attitude_euler_ = false;
    flag_attitude_quat_ = false;
    running_ = false;
}

MQTTServer::~MQTTServer()
{
    stop();
}

void MQTTServer::start()
{
    mqtt::connect_options connOpts;
    connOpts.set_clean_session(true);
    connOpts.set_keep_alive_interval(20);
    connOpts.set_automatic_reconnect(true);

    client_.connect(connOpts)->wait();
    client_.subscribe(sub_topic_, 1)->wait();
    std::cout << "[MQTT] 连接成功" << std::endl;

    running_ = true;
    sender_th_ = std::thread(&MQTTServer::sender_thread, this);
}

void MQTTServer::stop()
{
    running_ = false;
    if (sender_th_.joinable())
        sender_th_.join();
    
    client_.disconnect()->wait();
    std::cout << "[MQTT] 断开连接" << std::endl;
}

void MQTTServer::sender_thread()
{
    while (running_)
    {
        auto pkt = send_data();
        client_.publish(pub_topic_, reinterpret_cast<const char*>(pkt.data()), pkt.size(), 1, false);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}

void MQTTServer::message_arrived(mqtt::const_message_ptr msg)
{
    // std::string ascii = msg->to_string();
    // ascii.erase(std::remove_if(ascii.begin(), ascii.end(), ::isspace), ascii.end());
    //
    // std::vector<uint8_t> payload;
    // if (ascii.size() % 2) return;
    // for (size_t i = 0; i < ascii.size(); i += 2)
    //     payload.push_back(static_cast<uint8_t>(std::stoi(ascii.substr(i, 2), nullptr, 16)));
    //
    // std::cout << "[RX] " << ascii << std::endl;
    //
    // if (payload.size() < 5) return;
    // uint16_t header = (payload[0] << 8) | payload[1];
    // uint16_t length = (payload[2] << 8) | payload[3];
    // if (header != 0x5A47) return;
    // if (payload.size() != length + 4) return;
    //
    // uint8_t cmd = payload[4];
    // if (cmd == 0x05 && length == 4)
    // {
    //     uint8_t cs = checksum(payload.data() + 3, 4);
    //     if (cs != payload[7]) { std::cout << "checksum err\n"; return; }
    //     flag_balance_ = true;
    //     std::cout << "[自动调平] 平台ID=" << +payload[5] << std::endl;
    // }
    // else if (cmd == 0x02 && length == 0x21)
    // {
    //     uint8_t att_mode = payload[6];
    //     int16_t pi = (payload[7] << 8) | payload[8];
    //     int16_t ri = (payload[9] << 8) | payload[10];
    //     int16_t yi = (payload[11] << 8) | payload[12];
    //
    //     int32_t qi[4];
    //     for (int i = 0; i < 4; ++i)
    //         qi[i] = (payload[13 + 4*i] << 24) | (payload[14 + 4*i] << 16) |
    //                 (payload[15 + 4*i] << 8)  | payload[16 + 4*i];
    //
    //     uint8_t cs = checksum(payload.data() + 4, 32);
    //     if (cs != payload[36]) { std::cout << "checksum err\n"; return; }
    //
    //     AttitudeData d;
    //     d.pitch = pi / 100.0; d.roll = ri / 100.0; d.yaw = yi / 100.0;
    //     d.q0 = qi[0] / 100000.0; d.q1 = qi[1] / 100000.0;
    //     d.q2 = qi[2] / 100000.0; d.q3 = qi[3] / 100000.0;
    //
    //     if (att_mode == 0x01)
    //     {
    //         flag_attitude_euler_ = true;
    //         attitude_data_ = d;
    //         std::cout << "[姿态-欧拉] pitch=" << d.pitch
    //                   << " roll=" << d.roll << " yaw=" << d.yaw << std::endl;
    //     }
    //     else if (att_mode == 0x02)
    //     {
    //         flag_attitude_quat_ = true;
    //         attitude_data_ = d;
    //         std::cout << "[姿态-四元] q0=" << d.q0 << " q1=" << d.q1
    //                   << " q2=" << d.q2 << " q3=" << d.q3 << std::endl;
    //     }
    // }
}

std::vector<uint8_t> MQTTServer::send_data()
{
    // const uint16_t header = 0x5A47;
    // const uint16_t frame_len = 0x003C;   // 60
    // const uint8_t  tele_cmd = 0x00;
    // const uint8_t  plat_id  = 0x01;
    // const uint8_t  att_mode = 0x02;      // 四元数模式
    //
    // auto to_int16 = [](double v) -> int16_t {
    //     return static_cast<int16_t>(std::round(v * 100));
    // };
    // auto to_int32 = [](double v) -> int32_t {
    //     return static_cast<int32_t>(std::round(v * 100000));
    // };
    //
    // struct __attribute__((packed)) Pkt
    // {
    //     uint16_t header;
    //     uint16_t len;
    //     uint8_t  tele;
    //     uint8_t  plat;
    //     uint8_t  mode;
    //     int16_t  pitch, roll, yaw;
    //     int32_t  q0, q1, q2, q3;
    //     int16_t  wx, wy, wz;
    //     uint8_t  dir[3];
    //     uint16_t rpm[3];
    //     uint8_t  reserved[19];
    //     uint8_t  sum;
    // } p {};
    //
    // p.header = htons(header);
    // p.len    = htons(frame_len);
    // p.tele   = tele_cmd;
    // p.plat   = plat_id;
    // p.mode   = att_mode;
    //
    // p.pitch = htons(to_int16(pitch_));
    // p.roll  = htons(to_int16(roll_));
    // p.yaw   = htons(to_int16(yaw_));
    //
    // int32_t iq0 = to_int32(q0_), iq1 = to_int32(q1_), iq2 = to_int32(q2_), iq3 = to_int32(q3_);
    // p.q0 = htonl(iq0); p.q1 = htonl(iq1); p.q2 = htonl(iq2); p.q3 = htonl(iq3);
    //
    // p.wx = htons(to_int16(wx_));
    // p.wy = htons(to_int16(wy_));
    // p.wz = htons(to_int16(wz_));
    //
    // memcpy(p.dir, wheel_dirs_.data(), 3);
    // for (int i = 0; i < 3; ++i) p.rpm[i] = htons(static_cast<uint16_t>(wheel_rpms_[i]));
    //
    // memset(p.reserved, 0, 19);
    //
    // uint8_t* ptr = reinterpret_cast<uint8_t*>(&p);
    // ptr[60] = checksum(ptr + 4, 56);   // 字节5~63 对应偏移4~59
    //
    // return std::vector<uint8_t>(ptr, ptr + 61);
}
