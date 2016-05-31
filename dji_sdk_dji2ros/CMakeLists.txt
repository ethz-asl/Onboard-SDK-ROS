cmake_minimum_required(VERSION 2.8.3)
project(dji_sdk_dji2ros)

add_definitions(-std=c++11)

find_package(catkin_simple REQUIRED)

catkin_simple()
cs_install()
cs_export()
