//
// Created by msmlab on 2025/11/14.
//

#ifndef FAN_CONTROL_CODE_ADAPT_C_WHEEL_H
#define FAN_CONTROL_CODE_ADAPT_C_WHEEL_H

#include <vector>
#include <cstdint>
#include <cstring>
#include <chrono>
#include <thread>
#include <stdexcept>
#include <string>
#include "serial.h"

class Wheel
{
public:
    struct Status
    {
        uint8_t id = 0;
        float   rpm = 0.f;
        float   current_a = 0.f;
        float   duty = 0.f;
    };

private:
    MsMSerial& ser_;
    const uint16_t head_ = 0xFFFE;
    const uint16_t length_ = 0x0019;
    const uint8_t  cmd_id_ = 0x03;
    const uint8_t  device_id_ = 0x01;
    const uint8_t  crc_ = 0x0E;

    /* 睡眠 5 ms */
    void sleep5ms() { std::this_thread::sleep_for(std::chrono::milliseconds(5)); }

    /* 底层发送：可选 0x88 头 + 4 字节 CAN ID + 8 字节数据 */
    void sendFrame(uint32_t can_id, const std::vector<uint8_t>& data, bool add_header = false);

public:
    explicit Wheel(MsMSerial& serial) : ser_(serial) {}

    /* 设置转速（RPM）→ 新协议：前 4 字节大端 ERPM，后 4 字节 0x00 */
    void setSpeed(uint8_t wheel_id, float rpm);

    /* 设置电流（A），符号代表方向；reverse 可额外反向 */
    void setCurrent(uint8_t wheel_id, float current_a, bool reverse = false);

    /* 阻塞读取一帧状态反馈，帧头 0x88 0x00 0x00 0x09 */
    Status readStatus();

    /* 关闭串口 */
    void close() { if (ser_.isOpen()) ser_.close(); }
};

#endif //FAN_CONTROL_CODE_ADAPT_C_WHEEL_H