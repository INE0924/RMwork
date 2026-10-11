cmake_minimum_required(VERSION 3.8)
project(rm_color_cpp)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui videoio)

add_executable(camera_node src/camera_node.cpp)
ament_target_dependencies(camera_node rclcpp sensor_msgs std_msgs cv_bridge)
target_include_directories(camera_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(camera_node ${OpenCV_LIBS})

add_executable(color_detector_node src/color_detector_node.cpp)
ament_target_dependencies(color_detector_node rclcpp sensor_msgs cv_bridge)
target_include_directories(color_detector_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(color_detector_node ${OpenCV_LIBS})

install(TARGETS camera_node color_detector_node DESTINATION lib/${PROJECT_NAME})

ament_package()
