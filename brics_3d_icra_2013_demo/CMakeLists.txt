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

rosbuild_find_ros_package(brics_3d)
rosbuild_find_ros_package(brics_3d_ros)

FIND_PACKAGE(OpenSceneGraph COMPONENTS osgGA osgUtil osgViewer)
  
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
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_executable(scene_analyzer 
    src/scene_analyzer.cpp 
    src/SimpleSceneAnalysis.cpp 
    src/RoiManager.cpp 
    src/RoiAdapter.cpp 
    src/SceneAssociation.cpp 
    ${brics_3d_PACKAGE_PATH}/src/brics_3d/algorithm/segmentation/EuclideanClusteringPCL.cpp 
    ${brics_3d_ros_PACKAGE_PATH}/src/brics_3d_ros/WorldModelQueryServer.cpp
    ${brics_3d_ros_PACKAGE_PATH}/src/brics_3d_ros/SceneGraphROSCommunicator.cpp
    ${brics_3d_ros_PACKAGE_PATH}/src/brics_3d_ros/SceneGraphResentServer.cpp
)

IF(USE_OSG)
    target_link_libraries(scene_analyzer brics3d_world_model brics3d_util brics3d_algorithm brics3d_core  
        scanlib 
        6dslam_core 
        newmat_cmake    
        ${DL_LIB}
        flann_s
        ANN
        ${OSG_LIBRARY_DEBUG}
        ${OSGGA_LIBRARY_DEBUG}
        ${OSGVIEWER_LIBRARY_DEBUG}
     )
ELSE(USE_OSG)
    target_link_libraries(scene_analyzer brics3d_world_model brics3d_util brics3d_algorithm brics3d_core  
        scanlib 
        6dslam_core 
        newmat_cmake    
        ${DL_LIB}
        flann_s
        ANN
     )
ENDIF(USE_OSG)       

