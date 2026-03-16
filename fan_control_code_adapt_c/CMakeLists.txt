cmake_minimum_required(VERSION 3.12)
project(fan_control_code_adapt_c LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 添加第三方库，但排除示例程序
add_subdirectory(thirdparty/serialPro EXCLUDE_FROM_ALL)

find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

add_library(inih STATIC
        thirdparty/inih/ini.c
        thirdparty/inih/cpp/INIReader.cpp
)
target_include_directories(inih PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/inih
        ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/inih/cpp
)

# 收集所有源文件
file(GLOB ALL_SRC_FILES CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

# 过滤掉备份文件和测试文件
set(SRC_FILES ${ALL_SRC_FILES})
list(FILTER SRC_FILES EXCLUDE REGEX ".*_backup\\.cpp$")
list(FILTER SRC_FILES EXCLUDE REGEX ".*_simplified\\.cpp$")
list(FILTER SRC_FILES EXCLUDE REGEX ".*_test\\.cpp$")

# 打印源文件列表（调试用）
message(STATUS "Source files to compile:")
foreach(SRC_FILE ${SRC_FILES})
    get_filename_component(FILE_NAME ${SRC_FILE} NAME)
    message(STATUS "  ${FILE_NAME}")
endforeach()

# ✅ 先创建可执行文件 target（必须在 include/link 之前）
add_executable(${PROJECT_NAME} ${SRC_FILES})

set_target_properties(${PROJECT_NAME} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

# ✅ 你的 include
target_include_directories(${PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# ✅ 让 NokovSDKClient.h 识别 Linux（不改 SDK 头文件的最小办法）
target_compile_definitions(${PROJECT_NAME} PRIVATE _LINUX)

# ✅ 导入你本地的 nokov so（只用一种方式！）
add_library(nokov_sdk SHARED IMPORTED GLOBAL)
if (NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/lib/libnokov_sdk.so")
    message(FATAL_ERROR "libnokov_sdk.so not found at: ${CMAKE_CURRENT_SOURCE_DIR}/lib/libnokov_sdk.so")
endif()
set_target_properties(nokov_sdk PROPERTIES
        IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/lib/libnokov_sdk.so"
)

find_path(PAHO_MQTTCPP_INCLUDE_DIR
        NAMES mqtt/async_client.h
        PATH_SUFFIXES include
)
find_library(PAHO_MQTTCPP_LIBRARY
        NAMES paho-mqttpp3 libpaho-mqttpp3
)
find_library(PAHO_MQTTC_LIBRARY
        NAMES paho-mqtt3as libpaho-mqtt3as
)
if (NOT PAHO_MQTTCPP_INCLUDE_DIR OR NOT PAHO_MQTTCPP_LIBRARY OR NOT PAHO_MQTTC_LIBRARY)
    message(FATAL_ERROR "Paho MQTT C/C++ not found. Please install paho-mqttpp3 + paho-mqtt3as development packages (e.g. libpaho-mqttpp3-dev, libpaho-mqtt-dev).")
endif()

# ✅ 链接
target_link_libraries(${PROJECT_NAME} PRIVATE
        serialPro
        serialib
        inih
        Threads::Threads
        Eigen3::Eigen
        nokov_sdk
        ${PAHO_MQTTCPP_LIBRARY}
        ${PAHO_MQTTC_LIBRARY}
)

# ✅ 运行时能找到你项目里的 libnokov_sdk.so（不想配 LD_LIBRARY_PATH 就加这个）
set_target_properties(${PROJECT_NAME} PROPERTIES
        BUILD_RPATH "\$ORIGIN/../lib"
)

file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${CMAKE_CURRENT_SOURCE_DIR}/lib/libnokov_sdk.so"
                "${CMAKE_BINARY_DIR}/lib/libnokov_sdk.so"
)

target_include_directories(${PROJECT_NAME} PRIVATE
        ${PAHO_MQTTCPP_INCLUDE_DIR}
)

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/config.ini"
        "${CMAKE_BINARY_DIR}/bin/config.ini"
        COPYONLY)

# 模块化架构说明
message(STATUS "========================================")
message(STATUS "模块化架构已启用:")
message(STATUS "  - SystemController: 主控制系统")
message(STATUS "  - DataCollector: 数据采集模块")
message(STATUS "  - ControlModeManager: 控制模式管理")
message(STATUS "  - StatusPublisher: 状态发布模块")
message(STATUS "  - ConfigManager: 配置管理模块")
message(STATUS "========================================")
