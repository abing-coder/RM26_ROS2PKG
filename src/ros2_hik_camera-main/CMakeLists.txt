cmake_minimum_required(VERSION 3.8)
project(hik_camera)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Export compile commands for clangd
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#set
set(OpenVINO_DIR /home/ubuntu/桌面/Robomaster/openvino_toolkit_ubuntu22_2025.0.0.17942.1f68be9f594_x86_64/runtime/cmake)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/hik_camera_node.cpp
)

# find dependencies
find_package(OpenCV REQUIRED)
find_package(OpenVINO REQUIRED)
find_package(Eigen3 REQUIRED)


# Include directories
include_directories(${EIGEN3_INCLUDE_DIRS}
                    ${CMAKE_CURRENT_SOURCE_DIR}/include  
                    ${OpenCV_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIRS}
                    ${sensor_msgs_INCLUDE_DIRS}
                    ${geometry_msgs_INCLUDE_DIRS}
                    ${std_msgs_INCLUDE_DIRS}
                    /home/ubuntu/桌面/RM26/src/rm_auto_aim/detection/include
                    /home/ubuntu/桌面/RM26/src/rm_auto_aim/detection/Eigen
                    /home/ubuntu/桌面/RM26/src/rm_auto_aim/detection/TrackerHeader
                    /home/ubuntu/桌面/RM26/src/rm_auto_aim/detection/TrackerHeader
                    /home/ubuntu/桌面/RM26/src/recive_pkg/include
                    )




target_include_directories(${PROJECT_NAME} PUBLIC hikSDK/include ${OpenVINO_LIBRARIES})

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_directories(${PROJECT_NAME} PUBLIC hikSDK/lib/amd64)
  install(
    DIRECTORY hikSDK/lib/amd64
    DESTINATION lib
  )
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_directories(${PROJECT_NAME} PUBLIC hikSDK/lib/amd64)
  install(
    DIRECTORY hikSDK/lib/amd64
    DESTINATION lib
  )
else()
  message(FATAL_ERROR "Unsupport host system architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()

target_link_libraries(${PROJECT_NAME}
  FormatConversion
  MediaProcess
  MvCameraControl
  MVRender
  MvUsb3vTL
  openvino::runtime
  ${OpenCV_INCLUDE_DIRS}
  ${OpenVINO_LIBRARY_DIRS}
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN hik_camera::HikCameraNode
  EXECUTABLE ${PROJECT_NAME}_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_cpplint
    ament_cmake_uncrustify
  )
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
