//
// Created by msmlab on 2025/11/14.
//

#ifndef FAN_CONTROL_CODE_ADAPT_C_MASS_CENTER_BALANCER_H
#define FAN_CONTROL_CODE_ADAPT_C_MASS_CENTER_BALANCER_H

#include <array>
#include <vector>
#include <deque>
#include <chrono>
#include <cmath>
#include <optional>
#include <numeric>
#include <algorithm>
#include <iostream>

struct Attitude { double roll{}, pitch{}, yaw{}; };
struct AngularVel { double wx{}, wy{}, wz{}; };
struct WheelStatus { int id{}; double rpm{}; };
struct Torque { double tx{}, ty{}, tz{}; };

class MassCenterBalancer {
public:
    MassCenterBalancer();
};

#endif //FAN_CONTROL_CODE_ADAPT_C_MASS_CENTER_BALANCER_H