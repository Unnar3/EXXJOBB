cmake_minimum_required(VERSION 2.8.3)
project(exx_test_statistic)

set(CMAKE_CXX_FLAGS "-O4 -g -fPIC -std=c++0x -fpermissive")

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


add_executable(test_statistic src/test_statistic.cpp)
target_link_libraries(test_statistic ${catkin_LIBRARIES} ${QT_LIBRARIES} compression)
add_executable(test_statistic_clouds src/test_statistic_clouds.cpp)
target_link_libraries(test_statistic_clouds ${catkin_LIBRARIES} ${QT_LIBRARIES} compression )