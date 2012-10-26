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

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

OPTION(USE_OSG "Enable Open Scene Graph" OFF)    
IF(USE_OSG)
    FIND_PACKAGE(OpenSceneGraph COMPONENTS osgGA osgUtil osgViewer)
    ADD_DEFINITIONS(-DENABLE_OSG)
ENDIF(USE_OSG)  

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)

rosbuild_add_library(${PROJECT_NAME}
    src/brics_3d_ros/SceneGraphTypeCasts.cpp
    src/brics_3d_ros/WorldModelQueryServer.cpp 
    src/brics_3d_ros/SceneGraphROSCommunicator.cpp
    src/brics_3d_ros/SceneGraphResentServer.cpp 
)

rosbuild_add_executable(world_model_node
    src/brics_3d_ros/WorldModelQueryServer.cpp 
    src/brics_3d_ros/SceneGraphROSCommunicator.cpp
    src/brics_3d_ros/SceneGraphResentServer.cpp 
    src/examples/world_model_node.cpp
)
IF(USE_OSG)
    target_link_libraries(world_model_node brics3d_world_model brics3d_util brics3d_core ${OSG_LIBRARY_DEBUG} ${OSGGA_LIBRARY_DEBUG} ${OSGVIEWER_LIBRARY_DEBUG})
ELSE(USE_OSG)
    target_link_libraries(world_model_node brics3d_world_model brics3d_util brics3d_core)
ENDIF(USE_OSG) 

rosbuild_add_executable(world_model_duplication_node src/brics_3d_ros/WorldModelQueryServer.cpp src/brics_3d_ros/SceneGraphROSCommunicator.cpp src/examples/world_model_duplication_node.cpp)
IF(USE_OSG)
    target_link_libraries(world_model_duplication_node brics3d_world_model brics3d_util brics3d_core ${OSG_LIBRARY_DEBUG} ${OSGGA_LIBRARY_DEBUG} ${OSGVIEWER_LIBRARY_DEBUG})
ELSE(USE_OSG)
    target_link_libraries(world_model_duplication_node brics3d_world_model brics3d_util brics3d_core)
ENDIF(USE_OSG) 

rosbuild_add_executable(world_model_sample_queries src/examples/world_model_sample_queries.cpp)
rosbuild_add_executable(world_model_transform_updater src/examples/world_model_transform_updater.cpp)
rosbuild_add_executable(tf_to_scene_graph_node src/examples/tf_to_scene_graph_node.cpp)
rosbuild_add_executable(scene_objects_sample_query src/examples/scene_objects_sample_query.cpp)