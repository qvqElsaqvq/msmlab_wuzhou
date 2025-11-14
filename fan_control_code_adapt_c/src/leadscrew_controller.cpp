//
// Created by msmlab on 2025/11/14.
//

#include "leadscrew_controller.h"

void LeadScrewController::writeInt32BE(std::vector<uint8_t>& buf, size_t offset, int32_t val)
{
    buf[offset]     = (val >> 24) & 0xFF;
    buf[offset + 1] = (val >> 16) & 0xFF;
    buf[offset + 2] = (val >> 8)  & 0xFF;
    buf[offset + 3] =  val        & 0xFF;
}

void LeadScrewController::modeSet(uint8_t id)
{
    std::vector<uint8_t> data(13);
    data[0]  = 0x08;
    data[1]  = 0x00;
    data[2]  = 0x00;
    data[3]  = 0x00;
    data[4]  = id;
    data[5]  = 0x00;
    data[6]  = 0x1A;
    data[7]  = 0x02;
    data[8]  = 0x00;
    data[9]  = 0xD0;
    data[10] = 0x51;
    data[11] = 0x00;
    data[12] = 0x01;
    writeData(data);
}

void LeadScrewController::velocityAccelerationSet(uint8_t id)
{
    std::vector<uint8_t> data(13);
    data[0]  = 0x08;
    data[1]  = 0x00;
    data[2]  = 0x00;
    data[3]  = 0x00;
    data[4]  = id;
    data[5]  = 0x00;
    data[6]  = 0x1A;
    data[7]  = 0x1D;
    data[8]  = 0x08;
    data[9]  = 0x00;
    data[10] = 0x09;
    data[11] = 0x03;
    data[12] = 0x03;
    writeData(data);
}

void LeadScrewController::motorOn(uint8_t id)
{
    std::vector<uint8_t> data(13);
    data[0]  = 0x08;
    data[1]  = 0x00;
    data[2]  = 0x00;
    data[3]  = 0x00;
    data[4]  = id;
    data[5]  = 0x00;
    data[6]  = 0x1A;
    data[7]  = 0x00;
    data[8]  = 0x00;
    data[9]  = 0x01;
    data[10] = 0x00;
    data[11] = 0x00;
    data[12] = 0x01;
    writeData(data);
}

void LeadScrewController::motorOff(uint8_t id)
{
    std::vector<uint8_t> data(13);
    data[0]  = 0x08;
    data[1]  = 0x00;
    data[2]  = 0x00;
    data[3]  = 0x00;
    data[4]  = id;
    data[5]  = 0x00;
    data[6]  = 0x1A;
    data[7]  = 0x00;
    data[8]  = 0x00;
    data[9]  = 0x00;
    data[10] = 0x00;
    data[11] = 0x00;
    data[12] = 0x00;
    writeData(data);
}

void LeadScrewController::moveTo(const std::vector<int32_t>& location)
{
    std::vector<uint8_t> data(19, 0);
    data[0] = 0x5A;
    data[1] = 0x47;
    data[2] = 0x00;
    data[3] = 0x19;
    data[4] = 0x04;
    data[5] = 0x01;

    if (location.size() == 2)            // X/Y
    {
        writeInt32BE(data, 6, location[0]); // X
        writeInt32BE(data, 10, location[1]);// Y
        // 14~17 已默认 0
    }
    else if (location.size() == 1)       // Z
    {
        // 6~13 已默认 0
        writeInt32BE(data, 14, location[0]);// Z
    }

    data[18] = 0x0E;
    writeData(data);
}