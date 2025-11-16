//
// Created by mijiao on 2025/10/31.
//

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
                if (h.frame_head == 0xFF || h.frame_head == 0xFE) {
                    return ok;
                } else {
                    return sofError;
                }
            });
            registerChecker([](const Tail &t, const uint8_t *data, int s) -> int {
                if (t.checksum == 0) {
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
                case 0x0:
                    return sizeof(PlaneData);
                case 0x1:
                    return sizeof(CmdPlaneBasic);
                case 0x2:
                    return sizeof(CmdPlaneTrajectory);
                case 0x3:
                    return sizeof(CmdPlanePower);
                default:
                    return 1ull;
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