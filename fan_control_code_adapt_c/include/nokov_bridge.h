#pragma once
#include <string>

struct RigidPose
{
    int id = -1;
    double x = 0, y = 0, z = 0;      // 单位按 SDK 输出（你打印是 mm）
    double qx = 0, qy = 0, qz = 0, qw = 1;
    long long timestamp = 0;
    int frame = 0;
};

int  Nokov_Start(const char* server_ip);     // 连接并开始回调
void Nokov_Stop();                            // 断开
bool Nokov_GetPoseByName(const std::string& name, RigidPose& out); // 取最新刚体位姿
bool Nokov_GetPoseById(int id, RigidPose& out);
