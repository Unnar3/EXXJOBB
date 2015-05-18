cmake_minimum_required(VERSION 2.8.3)
project(test_ogre)

set(CMAKE_CXX_FLAGS "-O4 -g -fPIC -std=c++0x -fpermissive")

###################################################################
# ROS related stuff                                               #
###################################################################
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

###################################################################
# OGRE related stuff                                              #
###################################################################

set(CMAKE_MODULE_PATH "/usr/share/OGRE/cmake/modules;${CMAKE_MODULE_PATH}")
set(OGRE_SAMPLES_INCLUDEPATH "/usr/share/OGRE/Samples/Common/include/")

set(CMAKE_DEBUG_POSTFIX "_d")
set(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_BINARY_DIR}/dist")

find_package(OGRE REQUIRED)
find_package(OIS REQUIRED)

include_directories(
	include
	dist/bin
	dist/media 
	${catkin_INCLUDE_DIRS} 
	${Boost_INCLUDE_DIRS}
	${OIS_INCLUDE_DIRS}
	${OGRE_INCLUDE_DIRS}
	${OGRE_SAMPLES_INCLUDEPATH}
)

set(HDRS
	./include/BaseApplication.h
	./include/TutorialApplication.h
)
 
set(SRCS
	./src/BaseApplication.cpp
	./src/TutorialApplication.cpp
)

set(HDRStest
	./include/BaseApplication.h
	./include/testApplication.h
)
 
set(SRCStest
	./src/BaseApplication.cpp
	./src/testApplication.cpp
)

add_executable(testOgreApp WIN32 ${HDRS} ${SRCS})
target_link_libraries(testOgreApp compression ${OGRE_LIBRARIES} ${OIS_LIBRARIES} ${catkin_LIBRARIES} "/usr/lib/x86_64-linux-gnu/libboost_system.so.1.54.0" libvtkCommon.so libvtkFiltering.so libvtkGraphics.so)
set_target_properties(testOgreApp PROPERTIES DEBUG_POSTFIX _d)

add_executable(testApp WIN32 ${HDRStest} ${SRCStest}) 
set_target_properties(testApp PROPERTIES DEBUG_POSTFIX _d)
target_link_libraries(testApp compression ${OGRE_LIBRARIES} ${OIS_LIBRARIES} ${catkin_LIBRARIES} "/usr/lib/x86_64-linux-gnu/libboost_system.so.1.54.0")
 
#file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/dist/bin)
#file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/dist/media)
 

#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/dist/bin)
 
install(TARGETS testOgreApp
	RUNTIME DESTINATION bin
	CONFIGURATIONS All)

install(TARGETS testApp
	RUNTIME DESTINATION bin
	CONFIGURATIONS All)
