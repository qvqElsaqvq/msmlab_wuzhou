//
// Created by msmlab on 2025/11/14.
//

#ifndef FAN_CONTROL_CODE_ADAPT_C_LEADSCREW_CONTROLLER_H
#define FAN_CONTROL_CODE_ADAPT_C_LEADSCREW_CONTROLLER_H

#include <vector>
#include <cstdint>
#include <cstring>
#include <thread>
#include <chrono>
#include "serial.h"

class LeadScrewController
{
private:
    MsMSerial& ser_;

    /* 睡眠 5 ms */
    void sleep5ms() { std::this_thread::sleep_for(std::chrono::milliseconds(5)); }

    /* int32 → 大端 4 字节 */
    void writeInt32BE(std::vector<uint8_t>& buf, size_t offset, int32_t val);

    /* 底层写 + 5 ms 延时 */
    void writeData(const std::vector<uint8_t>& data)
    {
        if (ser_.isOpen())
        {
            ser_.write(data);
            sleep5ms();
        }
    }

public:
    explicit LeadScrewController(MsMSerial& serial) : ser_(serial) {}

    /* 设定为位置模式（相对位置） */
    void modeSet(uint8_t id);

    /* 设置加速度、减速度、最大速度 */
    void velocityAccelerationSet(uint8_t id);

    /* 使能电机 */
    void motorOn(uint8_t id);

    /* 失能电机 */
    void motorOff(uint8_t id);

    /*
     * 发送位置控制指令（相对位移）
     * location 长度 1 → 只动 Z 轴
     *          长度 2 → 动 X/Y 轴，Z 补 0
     */
    void moveTo(const std::vector<int32_t>& location);

    /* 关闭串口 */
    void close(){ if (ser_.isOpen()) ser_.close(); }
};

#endif //FAN_CONTROL_CODE_ADAPT_C_LEADSCREW_CONTROLLER_H