cmake_minimum_required(VERSION 2.8.3)
project(action_server)

find_package(catkin_simple REQUIRED)
# example boost usage


find_package(Boost REQUIRED COMPONENTS system thread)
find_package(geometry_msgs)


catkin_simple()
# Executables
cs_add_executable(action_server src/action_server.cpp)
cs_add_executable(action_client src/action_client.cpp)
cs_add_executable(action_lidar src/lidar_alarm.cpp)



cs_install()
cs_export()
    
