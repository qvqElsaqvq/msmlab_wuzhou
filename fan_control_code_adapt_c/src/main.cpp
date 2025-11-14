#include <iostream>
#include <thread>
#include <chrono>
#include <string>
#include <mutex>
#include <vector>
#include <Eigen/Dense>
#include "leadscrew_controller.h"
#include "wheel.h"
#include "fan.h"
#include "attitude_pd_controller.h"
#include "mass_center_balancer.h"
#include "serial.h"
#include "gyro_scope.h"
#include "MQTT_server.h"

struct Attitude
{
    double roll  = 0;
    double pitch = 0;
    double yaw   = 0;
};

struct AngularVel
{
    double wx = 0, wy = 0, wz = 0;
};

MsMSerial msm_serial("COM7", 115200);  //串口

/* ---------------- 线程任务 ---------------- */
void do_balance_task(MassCenterBalancer& balancer)
{
    std::cout << "执行自动调平算法...\n";
    std::vector<int32_t> action{-100000};
    balancer.leadscrew().move_to(action);   // 先降 Z 轴质量块
    balancer.balance_both_axes_fan();
    balancer.balance_z_axes_fan();
    std::cout << "调平完成\n";
}

void do_attitude_control_task(AttitudePDController& controller,
                              const Attitude& target)
{
    std::cout << "执行姿态控制算法...\n";
    controller.set_attitude_before_balancing(target);
    std::cout << "姿态控制完成\n";
}

/* ---------------- main ---------------- */
int main()
{
    asio::io_context io;
    GyroScope      gyro(io, "COM7", 115200);
    LeadScrewController leadscrew(io);
    Wheel   wheel(io);
    Fan    fan(io);

    AttitudePDController controller(wheel, gyro, fan);
    MassCenterBalancer balancer(leadscrew, gyro, wheel, controller);

    MQTTServer mqtt("192.168.31.81", 1883,
                    "satellite/data",
                    "satellite/cmd");
    mqtt.start();

    try
    {
        while (true)
        {
            /* 1. 调平指令 */
            if (mqtt.flag_balance.exchange(false))
            {
                std::thread([&balancer]{
                    do_balance_task(balancer);
                }).detach();
            }

            /* 2. 欧拉角姿态指令 */
            if (mqtt.flag_attitude_euler.exchange(false))
            {
                std::thread([&controller, &mqtt]{
                    do_attitude_control_task(controller,
                                             mqtt.attitude_data_euler);
                }).detach();
            }

            /* 3. 四元数姿态指令 */
            if (mqtt.flag_attitude_quat.exchange(false))
            {
                std::thread([&controller, &mqtt]{
                    auto q = mqtt.attitude_data_quat;
                    // quat -> euler ZYX
                    Eigen::Vector3d ea = q.toRotationMatrix().eulerAngles(2,1,0);
                    Attitude att{ea[2], ea[1], ea[0]};
                    do_attitude_control_task(controller, att);
                }).detach();
            }

            /* 4. 主循环更新传感器数据 */
            gyro.read_gyro();
            Attitude  att  = gyro.get_attitude();
            AngularVel av  = gyro.get_angular_velocity();
            Eigen::Quaterniond q =
                Eigen::AngleAxisd(att.yaw,   Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(att.pitch, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(att.roll,  Eigen::Vector3d::UnitX());

            double t = std::chrono::duration<double>(
                std::chrono::system_clock::now().time_since_epoch()).count();
            mqtt.publish_telemetry(t, att, av, q);

            controller.set_current_state(att, av);

            sleep_for(10ms);
        }
    }
    catch (std::exception& e)
    {
        cerr << "异常退出: " << e.what() << '\n';
    }

    return 0;
}