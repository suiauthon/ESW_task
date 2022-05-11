cmake_minimum_required(VERSION 2.8.3)
project(ESW_task)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    roslib
    rospack
    std_msgs
    message_generation
)

find_package(cmake_modules REQUIRED)
find_package(glog REQUIRED)

add_message_files(
  FILES
  ESWTask.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS message_runtime
   LIBRARIES ESW_task
   CATKIN_DEPENDS roscpp rospy roslib rospack
)

include_directories(
  include
  msg
  ${catkin_INCLUDE_DIRS}
)

add_executable(random_number_publisher src/random_number_publisher.cpp)
target_link_libraries(random_number_publisher ${catkin_LIBRARIES} glog::glog)

add_executable(random_number_subscriber src/random_number_subscriber.cpp)
target_link_libraries(random_number_subscriber ${catkin_LIBRARIES} glog::glog)
