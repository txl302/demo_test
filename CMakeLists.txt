cmake_minimum_required(VERSION 2.8.3)
project(demo_test)


find_package(catkin_simple REQUIRED)

catkin_simple()

cs_add_executable(robot_controller src/robot_controller.cpp)
cs_add_executable(robot_kpkv_serviceClient src/kpkv_serviceClient.cpp)
cs_add_executable(robot_trajectory_action_server src/robot_trajectory_action_server.cpp)
cs_add_executable(robot_trajectory_action_client src/robot_trajectory_action_client.cpp)

cs_install()
cs_export()