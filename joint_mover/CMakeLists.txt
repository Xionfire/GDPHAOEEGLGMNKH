cmake_minimum_required(VERSION 3.5)
project(joint_mover)


if(NOT CMAKE_CXX_STANDARD)
set(CMAKE_CXX_STANDARD 17)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
# AJOUTÉ : Dépendances pour le traitement XACRO et l'affichage du robot
find_package(xacro REQUIRED) 
find_package(robot_state_publisher REQUIRED)


add_executable(move_joints src/move_joints.cpp)
ament_target_dependencies(move_joints rclcpp sensor_msgs)


install(TARGETS move_joints
DESTINATION lib/${PROJECT_NAME})


# Installation des fichiers URDF et Launch
install(DIRECTORY launch urdf rviz
DESTINATION share/${PROJECT_NAME})


ament_package()