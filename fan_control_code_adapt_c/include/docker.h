//
// Created by mijiao on 2026/3/6.
//

#ifndef FAN_CONTROL_CODE_ADAPT_C_DOCK_H
#define FAN_CONTROL_CODE_ADAPT_C_DOCK_H
#include "attitude_pd_controller.h"
#include "gyro_scope.h"
#include "nokov_bridge.h"
#include "message.h"


class Docker {
private:
    AttitudePDController &controller_;
    GyroScope &gyro_;

public:
    Docker(GyroScope& gyro, AttitudePDController &controller);

    bool docking(CooperationDockData cooperation_dock_data);
};


#endif //FAN_CONTROL_CODE_ADAPT_C_DOCK_H
