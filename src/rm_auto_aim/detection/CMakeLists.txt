cmake_minimum_required(VERSION 3.20)
project(detection VERSION 0.1.0 LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


set(OpenVINO_DIR /home/ubuntu/桌面/Robomaster/openvino_toolkit_ubuntu22_2025.0.0.17942.1f68be9f594_x86_64/runtime/cmake)
find_package(OpenVINO REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
    ${OpenCV_INCLUDE_DIRS} 
    ${OpenVINO_INCLUDE_DIRS}
)

include_directories(./include)
include_directories(./TimeCounter)

file(GLOB_RECURSE HEADERS "src/*.cpp")
add_executable(${PROJECT_NAME} main.cpp ${HEADERS})
add_executable(test_aim_node test_aim_node.cpp ${HEADERS})
# message(${SOURCES})


target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include 
    ${OpenCV_INCLUDE_DIRS}
    ${OpenVINO_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    ${OpenVINO_LIBRARIES}  
    ${OpenCV_LIBS}              
    openvino::runtime
)

target_link_libraries(test_aim_node PRIVATE
    ${OpenVINO_LIBRARIES}  
    ${OpenCV_LIBS}              
    openvino::runtime
)
