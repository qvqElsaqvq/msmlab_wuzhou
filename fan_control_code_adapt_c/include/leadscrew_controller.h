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
    msmserial::MsMSerial& ser_;

    bool if_power_off_;

    // 当前质量块位置 [X, Y, Z]
    std::vector<int16_t> current_positions_{0, 0, 0};

public:
    explicit LeadScrewController(msmserial::MsMSerial& serial);

    /*
     * 发送位置控制指令（相对位移）
     * location 长度 1 → 只动 Z 轴
     *          长度 2 → 动 X/Y 轴，Z 补 0
     */
    void moveTo(const std::vector<int16_t>& location);

    /*
     * 获取当前质量块位置 [X, Y, Z]
     */
    const std::vector<int16_t>& getCurrentPositions() const;

    void setIfPowerOff( bool if_power_off );
};

#endif //FAN_CONTROL_CODE_ADAPT_C_LEADSCREW_CONTROLLER_H