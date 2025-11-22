//
// Created by mijiao on 2025/10/31.
//

#ifndef FAN_CONTROL_CODE_ADAPT_C_SERIAL_H
#define FAN_CONTROL_CODE_ADAPT_C_SERIAL_H

#include "message.h"

#include <iostream>
#include <iomanip>
#include <cmath>
#include "msg_serialize.h"

namespace msmserial
{
    class MsMSerial : public sp::serialPro<Head, Tail> {
    public:
        enum error {
            lengthNotMatch = -2, // 从下位机接受的消息长度与注册回调时传入的消息长度不匹配
            rxLessThanLength = -1, // 当前缓冲区中的消息不完整，下次解析时重试
            ok = 0,
            sofError, // 帧头不匹配
            crcError // crc8校验结果错误
        };

        // 构造函数
        MsMSerial() = default;

        MsMSerial(const std::string &port, int baud) : sp::serialPro<Head, Tail>(port, baud) {
            registerSetter([](Tail &t, const uint8_t *data, int s) {
                t.checksum = 0; // 异或校验
            });

            registerChecker([](const Head &h) -> int {
                if (h.frame_head == 0x6F || h.frame_head == 0x6E) {
                    return ok;
                } else {
                    return sofError;
                }
            });
            registerChecker([](const Tail &t, const uint8_t *data, int s) -> int {
                if (t.checksum == 0xED || t.checksum == 0xED) {
                    return ok;
                } else {
                    return crcError;
                }
            });
            setGetId([](const Head &h) {
                return h.command;
            });
            setGetLength([](const Head& h)
            {
                switch (h.command)
                {
                case 0x01:
                    return sizeof(FanControl);
                case 0x02:
                    return sizeof(WheelControl);
                case 0x03:
                    return sizeof(FanCalibrationControl);
                case 0x04:
                    return sizeof(LeadScrewControl);
                case 0x05:
                    return sizeof(GyroScopeData);
                case 0x06:
                    return sizeof(WheelData);
                case 0x07:
                    return sizeof(PowerData);
                case 0x08:
                    return sizeof(FanCalibrationData);
                case 0x09:
                    return sizeof(LeadScrewAlarm);
                default:
                    return 1ul;
                }
            });

            setListenerMaxSize(256);
        }

        MsMSerial(const MsMSerial &other) = delete;

        MsMSerial(MsMSerial &&other) noexcept : sp::serialPro<Head, Tail>(std::move(other)) {
        }

        using sp::serialPro<Head, Tail>::operator=;

        // 发送数据
        template<typename T>
        bool write(uint8_t id, const T &t) {
            return sp::serialPro<Head, Tail>::write(Head{.command = id}, t);
        }
    };
}

#endif //FAN_CONTROL_CODE_ADAPT_C_SERIAL_H
