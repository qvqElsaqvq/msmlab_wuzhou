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

# ✅ 先创建可执行文件 target（必须在 include/link 之前）
add_executable(${PROJECT_NAME} ${SRC_FILES})

# ✅ 你的 include
target_include_directories(${PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${EIGEN3_INCLUDE_DIRS}
)

# ✅ 让 NokovSDKClient.h 识别 Linux（不改 SDK 头文件的最小办法）
target_compile_definitions(${PROJECT_NAME} PRIVATE _LINUX)

# ✅ 导入你本地的 nokov so（只用一种方式！）
add_library(nokov_sdk SHARED IMPORTED GLOBAL)
set_target_properties(nokov_sdk PROPERTIES
        IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/lib/libnokov_sdk.so"
)

# ✅ 链接
target_link_libraries(${PROJECT_NAME} PRIVATE
        serialPro
        serialib
        inih
        Threads::Threads
        nokov_sdk
        paho-mqttpp3
        paho-mqtt3as
)

# ✅ 运行时能找到你项目里的 libnokov_sdk.so（不想配 LD_LIBRARY_PATH 就加这个）
set_target_properties(${PROJECT_NAME} PROPERTIES
        BUILD_RPATH "\$ORIGIN/../lib"
)
