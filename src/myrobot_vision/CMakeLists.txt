cmake_minimum_required(VERSION 3.8)
project(myrobot_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)


install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  myrobot_vision/aruco_detector.py
  myrobot_vision/detected_crates_tf.py
  myrobot_vision/relocalizer.py
  myrobot_vision/sequence_manager.py
  myrobot_vision/object_detection.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
