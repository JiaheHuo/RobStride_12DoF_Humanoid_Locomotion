cmake_minimum_required(VERSION 3.16)
project(robstride_usb2can_ctrl LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(yaml-cpp REQUIRED)
option(RS_DO_MOTOR_INIT "Run motor init sequence" ON)
option(RS_ENABLE_TELEMETRY "Enable telemetry printing" ON)
set(EIGEN3_INCLUDE_DIR
    "/home/jhuo/model_based_control/cyborg-test-static-arm/src/src/azureloong_control/third_party/eigen3"
    CACHE PATH "Eigen3 include dir"
)
add_compile_definitions(
  RS_DO_MOTOR_INIT=$<IF:$<BOOL:${RS_DO_MOTOR_INIT}>,1,0>
  RS_ENABLE_TELEMETRY=$<IF:$<BOOL:${RS_ENABLE_TELEMETRY}>,1,0>
)


include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/third_party/tangair_usb2can
)

add_executable(motor_test
  src/main.cpp
  src/config.cpp
  src/router.cpp
  src/motor.cpp
  src/stats.cpp
  src/ankle_kinematics.cpp
  src/GridRobot.cpp
  src/witmotion_imu.cpp
)
target_link_libraries(motor_test PRIVATE yaml-cpp pthread)
target_include_directories(motor_test PRIVATE
  ${EIGEN3_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
# ---- Tangair USB2CAN SDK: 选择一种方式 ----
# 方式A：如果你有 usb_can.c / usb_can.cpp 源码（推荐简单）
if (EXISTS "${PROJECT_SOURCE_DIR}/third_party/tangair_usb2can/usb_can.c")
  target_sources(motor_test PRIVATE third_party/tangair_usb2can/usb_can.c)
elseif (EXISTS "${PROJECT_SOURCE_DIR}/third_party/tangair_usb2can/usb_can.cpp")
  target_sources(motor_test PRIVATE third_party/tangair_usb2can/usb_can.cpp)
endif()

# 方式B：如果你只有 libusb_can.a 或 libusb_can.so
if (EXISTS "${PROJECT_SOURCE_DIR}/third_party/tangair_usb2can/libusb_can.a")
  target_link_libraries(motor_test PRIVATE ${PROJECT_SOURCE_DIR}/third_party/tangair_usb2can/libusb_can.a)
elseif (EXISTS "${PROJECT_SOURCE_DIR}/third_party/tangair_usb2can/libusb_can.so")
  target_link_libraries(motor_test PRIVATE ${PROJECT_SOURCE_DIR}/third_party/tangair_usb2can/libusb_can.so)
endif()

# 优化 & 警告
target_compile_options(motor_test PRIVATE -O2 -Wall -Wextra)
