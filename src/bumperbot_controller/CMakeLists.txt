cmake_minimum_required(VERSION 3.5)
project(bumperbot_controller)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})



ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY include
  DESTINATION include
)


install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)


ament_package()