cmake_minimum_required(VERSION 2.8.3)
project(exx_test_surfel)

set(CMAKE_CXX_FLAGS "-O4 -g -fPIC -std=c++0x -O4")

find_package(catkin REQUIRED COMPONENTS
  	sensor_msgs
	roscpp
	std_msgs
	image_transport
	message_generation
	pcl_conversions
  	pcl_ros
  	exx_compression
  	ransac_primitives
  	metaroom_xml_parser
  	qt_build
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
	CATKIN_DEPENDS message_runtime exx_compression qt_build libqt4-dev
)
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})


add_executable(surfel_test src/surfel_test.cpp)
target_link_libraries(surfel_test ${catkin_LIBRARIES} ${QT_LIBRARIES} compression)
