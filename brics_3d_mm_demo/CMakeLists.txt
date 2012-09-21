cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

rosbuild_find_ros_package(brics_3d_ros)
rosbuild_find_ros_package(brics_mm_ros)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

ADD_DEFINITIONS(-DBRICS_DATA_DIR=\\"${brics_mm_ros_PACKAGE_PATH}/brics_mm/data\\")
ADD_DEFINITIONS(-DBRICS_MODELS_DIR=\\"${brics_mm_ros_PACKAGE_PATH}/brics_mm/data/models\\")
ADD_DEFINITIONS(-DBRICS_OSG_ENABLE)

OPTION(USE_OSG "Enable Open Scene Graph" ON)    
IF(USE_OSG)
    FIND_PACKAGE(OpenSceneGraph COMPONENTS osgDB osgFX osgGA osgManipulator osgUtil osgViewer) #BRICS_MM
    ADD_DEFINITIONS(-DENABLE_OSG) #BRICS_3D
    ADD_DEFINITIONS(-DBRICS_OSG_ENABLE) #BRICS_MM
ENDIF(USE_OSG) 


INCLUDE_DIRECTORIES(${brics_mm_ros_PACKAGE_PATH}/src/)

rosbuild_add_executable(brics_3d_mm_demo
	src/main.cpp
	src/ROSWrapper.cpp
	src/SceneGraphToGeomConverter.cpp
	${brics_mm_ros_PACKAGE_PATH}/src/ompl/OmplWrapper.cpp
	${brics_3d_ros_PACKAGE_PATH}/src/brics_3d_ros/WorldModelQueryServer.cpp
)

target_link_libraries(brics_3d_mm_demo
    brics3d_world_model 
    brics3d_util 
    brics3d_core     
    BRICS_MM
    pqp
	qhull
	vclip
	yaobi
    ${OPENSCENEGRAPH_LIBRARIES}
)
