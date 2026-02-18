cmake_minimum_required(VERSION 3.8)
project(myrobot_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY behavior_tree config launch scripts
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  myrobot_navigation/simplepath.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
