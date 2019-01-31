cmake_minimum_required(VERSION 2.8.3)
project(jog_arm)

add_compile_options(-std=c++11)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
  jog_msgs
  moveit_ros_planning_interface
  rosparam_shortcuts
  tf
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    roscpp
    moveit_ros_manipulation
    moveit_ros_move_group
    moveit_ros_planning_interface
    tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(jog_arm_server src/jog_arm/jog_arm_server.cpp)
add_dependencies(jog_arm_server ${catkin_EXPORTED_TARGETS})
target_link_libraries(jog_arm_server ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

add_executable(spacenav_to_twist src/jog_arm/teleop_examples/spacenav_to_twist.cpp)
add_dependencies(spacenav_to_twist ${catkin_EXPORTED_TARGETS})
target_link_libraries(spacenav_to_twist ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

add_executable(xbox_to_twist src/jog_arm/teleop_examples/xbox_to_twist.cpp)
add_dependencies(xbox_to_twist ${catkin_EXPORTED_TARGETS})
target_link_libraries(xbox_to_twist ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

add_executable(dragonrise_to_twist src/jog_arm/teleop_examples/dragonrise_to_twist.cpp)
add_dependencies(dragonrise_to_twist ${catkin_EXPORTED_TARGETS})
target_link_libraries(dragonrise_to_twist ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

install(TARGETS jog_arm_server spacenav_to_twist xbox_to_twist dragonrise_to_twist
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

