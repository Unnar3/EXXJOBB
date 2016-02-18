cmake_minimum_required(VERSION 2.8.3)
project(exx_dataset_copy)

#set(CMAKE_CXX_FLAGS "-O4 -g -fPIC -std=c++0x")

find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
)

#add_message_files(FILES
#	distPose.msg
#)

#generate_messages(DEPENDENCIES
#	std_msgs
#)

catkin_package(
	DEPENDS
	CATKIN_DEPENDS message_runtime
   LIBRARIES
)
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})


add_executable(copy_waypoints_copy src/copy_waypoints.cpp)
target_link_libraries(copy_waypoints_copy ${catkin_LIBRARIES} ${QT_LIBRARIES})
