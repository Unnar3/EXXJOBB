cmake_minimum_required(VERSION 2.8.3)
project(exx_dataset)

set(CMAKE_CXX_FLAGS "-O4 -g -fPIC -std=c++0x")

find_package(catkin REQUIRED COMPONENTS
  	sensor_msgs
	roscpp
	std_msgs
  	pcl_ros
  	metaroom_xml_parser
  	qt_build
    tf_conversions
)

rosbuild_prepare_qt4(QtCore QtXml)

#add_message_files(FILES
#	distPose.msg
#)

#generate_messages(DEPENDENCIES
#	std_msgs
#)

catkin_package(
	DEPENDS
	CATKIN_DEPENDS message_runtime  qt_build libqt4-dev
   LIBRARIES
)
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})


add_executable(copy_waypoints src/copy_waypoints.cpp)
target_link_libraries(copy_waypoints ${catkin_LIBRARIES} ${QT_LIBRARIES})

add_executable(complete_lower src/complete_lower.cpp)
target_link_libraries(complete_lower ${catkin_LIBRARIES} ${QT_LIBRARIES})
