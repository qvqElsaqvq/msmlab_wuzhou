//
// Created by msmlab on 2025/11/14.
//

#include "fan.h"

void Fan::sendFrame(const std::array<uint8_t, 8> &payload) {
    std::array<uint8_t, 11> frame{};
    frame[0] = 0xFF;
    frame[1] = 0xFE;
    frame[2] = 0x00;
    std::copy(payload.begin(), payload.end(), frame.begin() + 3);

    uint8_t xorVal = 0;
    for (size_t i = 0; i < 10; ++i) xorVal ^= frame[i];
    frame[10] = xorVal;

    ser_.write(frame);
}

void Fan::setPWM(const std::array<uint8_t, 8> &pwm) {
    std::array<uint8_t, 8> data{};
    for (size_t i = 0; i < 8; ++i)
        data[i] = clampByte(pwm[i]) + 10; // 与 Python 保持一致
    sendFrame(data);
}

void Fan::setTorque(float tx, float ty, float tz) {
    uint8_t dir = 0;
    if (tx < 0) dir |= 1 << 2;
    if (ty < 0) dir |= 1 << 1;
    if (tz < 0) dir |= 1 << 0;

    auto toUint8_100 = [](float v) {
        int iv = static_cast<int>(std::fabs(v) * 100.0f);
        return static_cast<uint8_t>(std::min(255, std::max(0, iv)));
    };

    std::array<uint8_t, 8> data{};
    data[0] = dir;
    data[1] = toUint8_100(tx);
    data[2] = toUint8_100(ty);
    data[3] = toUint8_100(tz);
    // data[4..7] 保持 0
    sendFrame(data);
}
