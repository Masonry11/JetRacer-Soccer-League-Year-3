cmake_minimum_required(VERSION 3.0.2)
project(jetracer_direct)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF(NOT CMAKE_BUILD_TYPE)

add_message_files(
    FILES
    motor_msg.msg
    strategy_msg.msg
    objects_msg.msg
    objects_stamped_msg.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    CATKIN_DEPENDS
    message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_install_python(PROGRAMS
    scripts/detector.py
    scripts/motor_control.py
    scripts/turn_180.py
    scripts/strike.py
    scripts/strategy.py
    scripts/search.py
    scripts/defense.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
