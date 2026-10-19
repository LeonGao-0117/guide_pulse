cmake_minimum_required(VERSION 3.16)
project(guide_pulse VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)  # 定位 share 目录（默认二维码图片）
# aimdk_msgs 已拷贝内置 src/aimdk_msgs（来源 sdk_t1-v0.9.4.4，与 tts_demo
# 内置副本一致），TTS 服务定义 (PlayTts/CommonRequest 等) 全部以 SDK 为准。
find_package(aimdk_msgs REQUIRED)
# 导航 action 定义（已拷贝内置 src/liwei_nav_msgs，来源 t1_slam_nav_ws）
find_package(liwei_nav_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nlohmann_json REQUIRED)
# enroll 注册面 HTTP 客户端（RobotCar 协议 v0.9.0+，系统 libcurl）；
# 迎宾二维码下载校验用系统 OpenSSL（sha256，v0.22.0 P-036）
find_package(CURL REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(Threads REQUIRED)

# ---------------------------------------------------------------------------
# Vendored Paho MQTT C++（已复制到本工程 third_party/，静态链接，
# 来源：tts_demo/third_party/paho.mqtt.cpp 1.6.0，含 paho.mqtt.c 1.3.16）
# 路径可通过 cmake 参数覆盖：
#   colcon build --cmake-args -DPAHO_MQTT_CPP_DIR=/path/to/paho.mqtt.cpp
# ---------------------------------------------------------------------------
set(PAHO_MQTT_CPP_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/paho.mqtt.cpp" CACHE PATH
  "vendored paho.mqtt.cpp 源码目录（默认使用工程自带 third_party 副本）")
get_filename_component(PAHO_MQTT_CPP_DIR "${PAHO_MQTT_CPP_DIR}" ABSOLUTE)
if(NOT EXISTS "${PAHO_MQTT_CPP_DIR}/CMakeLists.txt")
  message(FATAL_ERROR "vendored Paho MQTT C++ 缺失: ${PAHO_MQTT_CPP_DIR}")
endif()
if(NOT EXISTS "${PAHO_MQTT_CPP_DIR}/externals/paho.mqtt.c/CMakeLists.txt")
  message(FATAL_ERROR "vendored paho.mqtt.c 缺失: ${PAHO_MQTT_CPP_DIR}/externals/paho.mqtt.c")
endif()

set(PAHO_BUILD_SHARED OFF CACHE BOOL "" FORCE)
set(PAHO_BUILD_STATIC ON CACHE BOOL "" FORCE)
set(PAHO_BUILD_SAMPLES OFF CACHE BOOL "" FORCE)
set(PAHO_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(PAHO_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(PAHO_ENABLE_TESTING OFF CACHE BOOL "" FORCE)
set(PAHO_WITH_MQTT_C ON CACHE BOOL "" FORCE)
set(PAHO_WITH_SSL ON CACHE BOOL "" FORCE)
add_subdirectory("${PAHO_MQTT_CPP_DIR}" "${CMAKE_BINARY_DIR}/_deps/paho_mqtt_cpp" EXCLUDE_FROM_ALL)

# ---------------------------------------------------------------------------
# 协议层：MQTT 业务报文 (JSON) 的解析与序列化 + enroll 注册面客户端 +
# 迎宾二维码缓存/下载（qr_code_cache，v0.22.0 P-036）+ 点位映射表本地
# 存储（point_map，v0.25.0 P-039），不依赖 ROS，可独立单测
# ---------------------------------------------------------------------------
add_library(guide_pulse_protocol STATIC
  src/guide_protocol.cpp
  src/robot_enroll.cpp
  src/qr_code_cache.cpp
  src/point_map.cpp
)
target_include_directories(guide_pulse_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(guide_pulse_protocol
  nlohmann_json::nlohmann_json
  CURL::libcurl
  OpenSSL::Crypto
)
target_compile_options(guide_pulse_protocol PRIVATE -Wall -Wextra)

# ---------------------------------------------------------------------------
# TTS 客户端 + 导航执行器 + MQTT 桥接节点
#（nav_executor/playlist/welcome/tour 为纯头文件，随使用方编译）
# ---------------------------------------------------------------------------
add_library(guide_pulse_core STATIC
  src/agent_monitor.cpp
  src/mc_transform.cpp
  src/screen_display.cpp
  src/tts_client.cpp
  src/mqtt_guide_bridge_node.cpp
)
target_include_directories(guide_pulse_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_options(guide_pulse_core PRIVATE -Wall -Wextra)
ament_target_dependencies(guide_pulse_core rclcpp ament_index_cpp aimdk_msgs liwei_nav_msgs rclcpp_action)
# 注意：与 ament_target_dependencies 保持一致的 plain 签名（勿加 PUBLIC/PRIVATE）
target_link_libraries(guide_pulse_core
  guide_pulse_protocol
  paho-mqttpp3-static
  Threads::Threads
)

add_executable(guide_bridge src/main.cpp)
target_compile_options(guide_bridge PRIVATE -Wall -Wextra)
# guide_pulse_core 已 PUBLIC 带出 protocol/paho/Threads 的全部传递依赖
target_link_libraries(guide_bridge guide_pulse_core)

# ---------------------------------------------------------------------------
# 安装
# ---------------------------------------------------------------------------
install(TARGETS guide_bridge
  DESTINATION lib/${PROJECT_NAME}
)
# Mock TTS 服务脚本（ros2 run guide_pulse mock_tts_service.py）
# Mock 运控服务脚本（迎宾变形联调：SetMcAction/GetMcAction 模拟）
install(PROGRAMS scripts/mock_tts_service.py scripts/mock_mc_service.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# ---------------------------------------------------------------------------
# 单元测试
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # 1. 协议解析/校验/序列化单测（无 ROS 依赖）
  ament_add_gtest(guide_protocol_test test/test_guide_protocol.cpp)
  target_link_libraries(guide_protocol_test guide_pulse_protocol)

  # 1b. enroll 注册面响应解析/错误码映射/串号校验单测（无 ROS 依赖，
  #     HTTP 传输层由集成测试经 enroll_mock_server 覆盖）
  ament_add_gtest(robot_enroll_test test/test_robot_enroll.cpp)
  target_link_libraries(robot_enroll_test guide_pulse_protocol)

  # 1c. 迎宾二维码缓存/下载纯函数单测（sha256 摘要/图片魔数/qr_sha256
  #     格式/缓存写入与淘汰；HTTP 传输层与模拟器联调覆盖）
  ament_add_gtest(qr_code_cache_test test/test_qr_code_cache.cpp)
  target_link_libraries(qr_code_cache_test guide_pulse_protocol)

  # 1d. 点位映射表本地存储单测（v0.25.0 P-039）：全量替换/原子落盘/
  #     跨实例加载/name->code 解析/损坏文件防御
  ament_add_gtest(point_map_test test/test_point_map.cpp)
  target_link_libraries(point_map_test guide_pulse_protocol)

  # 2. TTS 客户端单测：进程内拉起 Mock PlayTts 服务端，
  #    校验请求字段映射与回执判定逻辑
  ament_add_gtest(tts_client_test test/test_tts_client.cpp src/tts_client.cpp)
  target_include_directories(tts_client_test PRIVATE include)
  ament_target_dependencies(tts_client_test rclcpp aimdk_msgs)
  target_link_libraries(tts_client_test guide_pulse_protocol Threads::Threads)

  # 3. 展厅讲解增量模块单测（纯 C++：播报状态机/讲解状态机）
  #（welcome_test 已随 welcome_default_texts 移除而删除：2026-10-19
  #  迎宾词强制映射表权威，make_welcome_texts 兜底链不复存在）
  ament_add_gtest(playlist_test test/test_playlist.cpp)
  target_link_libraries(playlist_test guide_pulse_protocol Threads::Threads)

  # 3c. 迎宾变形纯函数单测：动作转移路径表/趴姿集合/reason 人话映射的
  #     不变式（路径逐条抄自 SDK _ACTION_RULES，抄错一格 = 机器人做出
  #     没预期的动作，靠测试锁住表结构）
  ament_add_gtest(mc_transform_test test/test_mc_transform.cpp
    src/mc_transform.cpp)
  target_include_directories(mc_transform_test PRIVATE include)
  ament_target_dependencies(mc_transform_test rclcpp aimdk_msgs)
  target_link_libraries(mc_transform_test Threads::Threads)

  # 3b. 屏幕显示纯函数单测：base64 解码（含非法输入）与 ffmpeg 命令拼装。
  #     ScreenDisplay 的服务调用/推送链路属真机集成，由 scripts 现场验证
  ament_add_gtest(screen_display_test test/test_screen_display.cpp
    src/screen_display.cpp)
  target_include_directories(screen_display_test PRIVATE include)
  ament_target_dependencies(screen_display_test rclcpp aimdk_msgs)

  ament_add_gtest(tour_test test/test_tour.cpp)
  target_link_libraries(tour_test guide_pulse_protocol Threads::Threads)

  # 3c2. 语料归一化/匹配纯函数单测（AgentMonitor 语料触发核心逻辑：
  # 标点/空白/大小写归一与子串命中不变式，锁住匹配行为）
  ament_add_gtest(agent_monitor_test test/test_agent_monitor.cpp
    src/agent_monitor.cpp)
  target_include_directories(agent_monitor_test PRIVATE include)
  ament_target_dependencies(agent_monitor_test rclcpp aimdk_msgs)
  target_link_libraries(agent_monitor_test Threads::Threads)

  # 4. 真机导航执行器单测：进程内拉起 Mock Navigate action 服务端
  #    （liwei_nav_msgs/action/Navigate + query 服务），校验受理/终态/取消逻辑
  ament_add_gtest(waypoint_nav_executor_test test/test_waypoint_nav_executor.cpp)
  target_include_directories(waypoint_nav_executor_test PRIVATE include)
  ament_target_dependencies(waypoint_nav_executor_test rclcpp rclcpp_action liwei_nav_msgs)
  target_link_libraries(waypoint_nav_executor_test guide_pulse_protocol Threads::Threads)
endif()

ament_package()
