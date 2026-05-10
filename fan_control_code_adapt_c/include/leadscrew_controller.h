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
    msmserial::MsMSerial& ser_;            // 串口对象（下发丝杆控制帧）

    bool if_power_off_;                    // 停机标志：true 时禁止输出

    std::vector<int16_t> last_step_{0, 0, 0}; // 最近一次下发的相对步长（用于状态回显/诊断）

public:
    // 构造：注册报警回调（如撞边），初始化状态
    explicit LeadScrewController(msmserial::MsMSerial& serial);

    /*
     * 发送位置控制指令（相对位移）
     * location 长度 1 → 只动 Z 轴
     *          长度 2 → 动 X/Y 轴，Z 补 0
     */
    void moveTo(const std::vector<int16_t>& location);

    // 获取最近一次下发/估计的位置（按 X/Y/Z 顺序）
    const std::vector<int16_t>& getCurrentPositions() const;

    // 设置停机标志
    void setIfPowerOff( bool if_power_off );
};

#endif //FAN_CONTROL_CODE_ADAPT_C_LEADSCREW_CONTROLLER_H
