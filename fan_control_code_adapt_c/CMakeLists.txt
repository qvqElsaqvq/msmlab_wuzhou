cmake_minimum_required(VERSION 3.22.1)
project(fan_control_code_adapt_c LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(thirdparty/serialPro)

find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(inih STATIC
        thirdparty/inih/ini.c
        thirdparty/inih/cpp/INIReader.cpp
)
target_include_directories(inih PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/inih
        ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/inih/cpp
)

file(GLOB SRC_FILES CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

set(VICON_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/vicon_datastream_sdk/Linux64" CACHE PATH "Vicon DataStream SDK directory")

# ✅ 先创建可执行文件 target（必须在 include/link 之前）
add_executable(${PROJECT_NAME} ${SRC_FILES})

# ✅ 你的 include
target_include_directories(${PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${VICON_SDK_DIR}
        ${EIGEN3_INCLUDE_DIRS}
)

add_library(vicon_sdk_cpp SHARED IMPORTED GLOBAL)
set_target_properties(vicon_sdk_cpp PROPERTIES
        IMPORTED_LOCATION "${VICON_SDK_DIR}/libViconDataStreamSDK_CPP.so"
)

# ✅ 链接
target_link_libraries(${PROJECT_NAME} PRIVATE
        serialPro
        serialib
        inih
        Threads::Threads
        vicon_sdk_cpp
        paho-mqttpp3
        paho-mqtt3as
)

# ✅ 运行时能找到 Vicon SDK 的 .so（不想配 LD_LIBRARY_PATH 就加这个）
set_target_properties(${PROJECT_NAME} PROPERTIES
        BUILD_RPATH "${VICON_SDK_DIR};\$ORIGIN/../lib"
)
