//
// Created by msmlab on 2025/11/14.
//

#include "wheel.h"

void Wheel::sendFrame(uint32_t can_id, const std::vector<uint8_t>& data, bool add_header = false)
{
    if (data.size() != 8)
        throw std::invalid_argument("data must be 8 bytes");

    std::vector<uint8_t> frame;
    if (add_header) frame.push_back(0x88);

    /* CAN ID 大端 4 字节 */
    frame.push_back(static_cast<uint8_t>(can_id >> 24));
    frame.push_back(static_cast<uint8_t>(can_id >> 16));
    frame.push_back(static_cast<uint8_t>(can_id >> 8));
    frame.push_back(static_cast<uint8_t>(can_id));

    frame.insert(frame.end(), data.begin(), data.end());
    ser_.write(frame);
}

void Wheel::setSpeed(uint8_t wheel_id, float rpm)
{
    uint32_t can_id = 0x00000300 + (wheel_id & 0xFF);
    int32_t  erpm   = static_cast<int32_t>(rpm);

    std::vector<uint8_t> data(8, 0);
    data[0] = static_cast<uint8_t>(erpm >> 24);
    data[1] = static_cast<uint8_t>(erpm >> 16);
    data[2] = static_cast<uint8_t>(erpm >> 8);
    data[3] = static_cast<uint8_t>(erpm);
    // 4~7 保持 0

    sendFrame(can_id, data, true);
}

void Wheel::setCurrent(uint8_t wheel_id, float current_a, bool reverse = false)
{
    const std::vector<uint8_t> header{0xFF, 0xFE};
    uint8_t id = wheel_id & 0xFF;

    bool dir_neg = (current_a < 0) ^ reverse;
    uint8_t direction = dir_neg ? 0xAA : 0x55;

    int16_t current_ma = static_cast<int16_t>(current_a * 100.f); // 1 A = 100
    uint8_t hi = (current_ma >> 8) & 0xFF;
    uint8_t lo = current_ma & 0xFF;

    uint8_t chk = 0;
    for (uint8_t b : {header[0], header[1], id, direction, hi, lo})
        chk ^= b;

    std::vector<uint8_t> frame{header[0], header[1], id, direction, hi, lo, chk};
    ser_.write(frame);
}

Status Wheel::readStatus()
{
    const std::vector<uint8_t> header{0x88, 0x00, 0x00, 0x09};
    std::vector<uint8_t> sync(4);

    /* 逐字节对齐帧头 */
    while (true)
    {
        size_t n = ser_.read(sync, 1);
        if (n == 0) throw std::runtime_error("串口无数据");
        if (sync[0] == header[0])
        {
            n = ser_.read(sync.data() + 1, 3);
            if (n == 3 && memcmp(sync.data(), header.data(), 4) == 0)
                break;
        }
    }

    /* 读剩余 9 字节 */
    std::vector<uint8_t> rest(9);
    size_t n = ser_.read(rest, 9);
    if (n != 9) throw std::runtime_error("接收数据不完整");

    uint8_t id = rest[0];
    const uint8_t* d = rest.data() + 1;

    int32_t rpm_raw     = (static_cast<int32_t>(d[0]) << 24) |
                          (static_cast<int32_t>(d[1]) << 16) |
                          (static_cast<int32_t>(d[2]) << 8)  |
                          static_cast<int32_t>(d[3]);
    int16_t current_raw = (static_cast<int16_t>(d[4]) << 8) | d[5];
    int16_t duty_raw    = (static_cast<int16_t>(d[6]) << 8) | d[7];

    Status st;
    st.id        = id;
    st.rpm       = rpm_raw / 7.0f;
    st.current_a = current_raw / 10.0f;
    st.duty      = duty_raw / 1000.0f;
    return st;
}