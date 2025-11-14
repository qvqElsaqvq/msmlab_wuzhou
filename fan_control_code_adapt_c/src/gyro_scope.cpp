//
// Created by msmlab on 2025/11/14.
//

#include "gyro_scope.h"

size_t GyroScope::readGyro() {
    /* 1. 把串口当前可读数据全部吸入 buf */
    size_t n = ser_.available();
    if (n) {
        std::vector<uint8_t> chunk(n);
        ser_.read(chunk);
        buf_.insert(buf_.end(), chunk.begin(), chunk.end());
    }

    /* 2. 循环找帧头并解析 */
    size_t parsed = 0;
    while (parsed < MAX_PER_CALL) {
        auto it = std::search(buf_.begin(), buf_.end(),
                              std::begin(HEADER), std::end(HEADER));
        if (it == buf_.end()) {
            /* 保留最后 1 字节防跨界 */
            if (buf_.size() > 1) {
                uint8_t last = buf_.back();
                buf_.clear();
                buf_.push_back(last);
            }
            break;
        }

        size_t offset = std::distance(buf_.begin(), it);
        if (buf_.size() < offset + SHORT_FRAME_LEN)
            break; // 数据不够一帧

        /* 提取完整帧 */
        std::array<uint8_t, SHORT_FRAME_LEN> frame{};
        std::copy_n(it, SHORT_FRAME_LEN, frame.begin());
        buf_.erase(buf_.begin(), it + SHORT_FRAME_LEN);

        /* 异或校验 */
        uint8_t xorSum = 0;
        for (size_t i = 0; i < frame.size() - 1; ++i) xorSum ^= frame[i];
        if (xorSum != frame.back()) continue; // 校验失败丢弃

        /* 解析 payload */
        const uint8_t *data = frame.data() + 3; // 跳过 2 头 + 1 长度
        if (data[0] != 0x00) continue; // 设备编号不符

        Vec3 av, at;
        av.x = bytesToShort(data[1], data[2]) / 100.0;
        av.y = bytesToShort(data[3], data[4]) / 100.0;
        av.z = bytesToShort(data[5], data[6]) / 100.0;

        at.z = bytesToShort(data[7], data[8]) / 100.0; // yaw
        at.y = bytesToShort(data[9], data[10]) / 100.0; // pitch
        at.x = bytesToShort(data[11], data[12]) / 100.0; // roll

        /* 线程安全写入 */
        {
            std::lock_guard<std::mutex> lg(mtx_);
            latestAngularVel_ = av;
            latestAttitude_ = at;
        }
        ++parsed;
    }
    return parsed;
}
