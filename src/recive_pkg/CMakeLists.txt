cmake_minimum_required(VERSION 3.8)
project(recive_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 设置策略以支持 IN_LIST 操作符
if(POLICY CMP0057)
  cmake_policy(SET CMP0057 NEW)
endif()

#set
set(OpenVINO_DIR /opt/intel/openvino_2024.6.0/cmake)
# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(SRC
      /home/ubuntu/桌面/Robomaster/RM26_ROS2PKG/src/rm_auto_aim/detection/src/yolo_detection.cpp
      /home/ubuntu/桌面/Robomaster/RM26_ROS2PKG/src/rm_auto_aim/detection/src/traditional_detector.cpp
      /home/ubuntu/桌面/Robomaster/RM26_ROS2PKG/src/rm_auto_aim/detection/src/openvino_profiler.cpp
      /home/ubuntu/桌面/Robomaster/RM26_ROS2PKG/src/rm_auto_aim/detection/src/inference_engine.cpp)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenVINO REQUIRED COMPONENTS Runtime)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui videoio dnn)
find_package(cv_bridge REQUIRED)


# Include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include  
                    ${OpenCV_INCLUDE_DIRS}
                    ${sensor_msgs_INCLUDE_DIRS}
                    ${geometry_msgs_INCLUDE_DIRS}
                    ${std_msgs_INCLUDE_DIRS}
                    /home/ubuntu/桌面/Robomaster/RM26_ROS2PKG/src/rm_auto_aim/detection/include
                    /home/ubuntu/桌面/Robomaster/RM26_ROS2PKG/src/recive_pkg/include
                    /home/ubuntu/桌面/Robomaster/RM26_ROS2PKG/src/rm_auto_aim/detection/TimeCounter
                    )



#add_executable
add_executable(${PROJECT_NAME}
  src/image_subscirber_node.cpp
  main.cpp
  ${SRC}
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  openvino::runtime
)

# 自动处理ROS2依赖与导入库
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  cv_bridge
)

# 仅链接必要的第三方库，避免拉入大量不必要模块
target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  openvino::runtime
)

# 按需链接，避免无用的大量 OpenCV 模块被强行拉入
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_link_options(${PROJECT_NAME} PRIVATE -Wl,--as-needed -Wl,--no-undefined)
endif()


# 安装目标到lib目录
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
