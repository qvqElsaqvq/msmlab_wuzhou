cmake_minimum_required(VERSION 3.22.1)
project(fan_control_code_adapt_c)

set(CMAKE_CXX_STANDARD 17)

add_subdirectory(thirdparty/serialPro)

find_package(Threads REQUIRED)
find_package(PahoMqttCpp REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(inih STATIC
        thirdparty/inih/ini.c
        thirdparty/inih/cpp/INIReader.cpp
)

include_directories(
        ${EIGEN3_INCLUDE_DIR}
        include
)

file(GLOB SRC_FILES CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

target_include_directories(inih PUBLIC thirdparty/inih)

add_executable(fan_control_code_adapt_c
        src/main.cpp
        src/gyro_scope.cpp
        src/fan.cpp
        src/attitude_pd_controller.cpp
        src/MQTT_server.cpp
        src/leadscrew_controller.cpp
        src/mass_center_balancer.cpp
        src/wheel.cpp
)
target_link_libraries(fan_control_code_adapt_c serialPro pthread paho-mqttpp3 paho-mqtt3as inih)