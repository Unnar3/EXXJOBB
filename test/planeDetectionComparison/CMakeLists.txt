cmake_minimum_required(VERSION 2.8.3)
project(planeDetectionComparison)

set(CMAKE_CXX_FLAGS "-O4 -g -fPIC -std=c++0x -O4")

find_package(catkin REQUIRED COMPONENTS
  	sensor_msgs
	roscpp
   ProbabilisticPrimitiveRefinement
	std_msgs
	image_transport
	message_generation
	pcl_conversions
  	pcl_ros
  	exx_compression
   ransac_primitives
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
	CATKIN_DEPENDS message_runtime exx_compression qt_build libqt4-dev
   LIBRARIES
)
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})


add_executable(comparison src/comparison.cpp)
target_link_libraries(comparison ${catkin_LIBRARIES} ${QT_LIBRARIES} compression)

add_executable(createTestCloud src/createTestCloud.cpp)
target_link_libraries(createTestCloud ${catkin_LIBRARIES})
