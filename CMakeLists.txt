cmake_minimum_required(VERSION 2.8.3)
project(AutoPTP_generater)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs message_generation)

## Add in message and service files
add_message_files(FILES raven_automove.msg raven_state.msg)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime)

## Build talker and listeneri
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(talkerAutoPTP src/talker.cpp  src/Raven_Controller.cpp src/Raven_PathPlanner.cpp)
target_link_libraries(talkerAutoPTP ${catkin_LIBRARIES})
add_dependencies(talkerAutoPTP AutoPTP_test_generate_messages_cpp)

add_executable(listenerAutoPTP src/listener.cpp)
target_link_libraries(listenerAutoPTP ${catkin_LIBRARIES})
add_dependencies(listenerAutoPTP AutoPTP_test_generate_messages_cpp)
