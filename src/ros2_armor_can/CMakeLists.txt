
cmake_minimum_required(VERSION 3.5)
project(ros2_armor_can)
#export LD_LIBRARY_PATH=/home/ubuntu/下载/0915\(2\)/0915/yolov5-onnxruntime/RM_1_8_8/ros2_armor_can/lib/bin/unix64/release:$LD_LIBRARY_PATH
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# bmcan 相关路径（如有需要可自行调整）
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(BMCAN_INCLUDE_DIR "./lib/bin/unix64/release/")
    set(BMCAN_LIB_NAME "bmapi64")
else()
    set(BMCAN_INCLUDE_DIR "./lib/bin/unix32/release")
    set(BMCAN_LIB_NAME "bmapi")
endif()

# 依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 头文件路径
include_directories(
  include
  ./lib/include
  ./include/robotcontrol
)
link_directories(${BMCAN_INCLUDE_DIR})

# armor_send 节点
add_executable(armor_send_node src/armor_send.cpp src/bmcan_bus.cpp)
add_executable(test_send_node src/test.cpp src/bmcan_bus.cpp)

ament_target_dependencies(armor_send_node rclcpp std_msgs geometry_msgs)
ament_target_dependencies(test_send_node rclcpp std_msgs geometry_msgs)
target_link_libraries(armor_send_node ${BMCAN_LIB_NAME})
target_link_libraries(test_send_node ${BMCAN_LIB_NAME})
install(TARGETS
  armor_send_node
  test_send_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装launch文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()


#source install/setup.bash
#export LD_LIBRARY_PATH=/home/ubuntu/下载/0915\(2\)/0915/yolov5-onnxruntime/RM_1_8_8/ros2_armor_can/lib/bin/unix64/release:$LD_LIBRARY_PATH
#./install/ros2_armor_can/lib/ros2_armor_can/armor_send_node

