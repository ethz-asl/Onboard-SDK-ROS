cmake_minimum_required(VERSION 2.8.3)
project(dji_waypoint_navigator)

add_definitions(-std=c++11)

find_package(catkin_simple REQUIRED)

catkin_simple()

cs_add_executable(dji_waypoint_navigator src/waypoint_navigator.cpp)
cs_add_executable(dji_test_navigator src/test_navigator.cpp)

add_dependencies(dji_waypoint_navigator ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(dji_waypoint_navigator ${catkin_LIBRARIES})
add_dependencies(dji_test_navigator ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(dji_test_navigator ${catkin_LIBRARIES})

cs_install()
cs_export()
