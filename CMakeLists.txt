cmake_minimum_required(VERSION 2.8.3)
project(lar_vision)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  pcl_ros
  roscpp
  rospy
)

#PCL
find_package(PCL 1.7 REQUIRED  )
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


#OpenCV
find_package(OpenCV REQUIRED)
if (OPENCV_FOUND)
  set(HAVE_OPENCV ON)
  include_directories(SYSTEM ${OPENCV_INCLUDE_DIRS})
  message("OPENCV INCLUDE DIR:")
  message(${OpenCV_LIBS})
endif(OPENCV_FOUND)

#Aruco
set(ARUCO_LIBS /usr/local/lib/libaruco.so)

#OpenMP
find_package(OpenMP REQUIRED)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

#Lar Tools
MESSAGE("#### LAR TOOLS #####")
find_package(lar_tools)
include_directories(${lar_tools_INCLUDE_DIRS})


##########################################################################

#Includes
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CATKIN_DEVEL_PREFIX}/include
  include/${PROJECT_NAME}/
  include/${PROJECT_NAME}/commons
  include/${PROJECT_NAME}/segmentation
  include/${PROJECT_NAME}/grasping
  include/${PROJECT_NAME}/grasping/grippers
  include/${PROJECT_NAME}/reconstruction
)


#Configurations
generate_dynamic_reconfigure_options(
  cfg/match_clouds.cfg
  cfg/camera_error_reduction.cfg
)

#Packaging
catkin_package(
  INCLUDE_DIRS
    include/
    include/${PROJECT_NAME}/
    include/${PROJECT_NAME}/commons
    include/${PROJECT_NAME}/segmentation
    include/${PROJECT_NAME}/grasping
    include/${PROJECT_NAME}/grasping/grippers
    include/${PROJECT_NAME}/reconstruction
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS cv_bridge image_transport pcl_ros roscpp rospy
#  DEPENDS system_lib
)


#LAR VISION COMMONS LIBRARY
add_library(lar_vision
        src/commons/lar_vision_commons.cpp
        src/commons/Noiser.cpp
        src/segmentation/HighMap.cpp
        src/segmentation/OnePointRansac.cpp
        src/grasping/Slicer.cpp
        src/grasping/Grasper.cpp
        src/grasping/grippers/GraspingGripper.cpp
        src/grasping/grippers/CrabbyGripper.cpp
        src/reconstruction/tsdf/marching_cubes_tsdf_octree.cpp
        src/reconstruction/tsdf/octree.cpp
        src/reconstruction/tsdf/tsdf_interface.cpp
        src/reconstruction/tsdf/tsdf_volume_octree.cpp
)
target_link_libraries (lar_vision ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} lar_tools)



#NODES

# planar_grasp_test
#add_executable(${PROJECT_NAME}_planar_grasp_test  src/nodes/planar_grasp_test.cpp)
#target_link_libraries(${PROJECT_NAME}_planar_grasp_test ${catkin_LIBRARIES} lar_vision lar_tools)

# match_clouds
#add_executable(${PROJECT_NAME}_match_clouds  src/nodes/match_clouds.cpp)
#target_link_libraries(${PROJECT_NAME}_match_clouds ${catkin_LIBRARIES} lar_vision lar_tools)
#add_dependencies(${PROJECT_NAME}_match_clouds ${PROJECT_NAME}_gencfg)

# _camera_error_reduction
#add_executable(${PROJECT_NAME}_camera_error_reduction  src/nodes/camera_error_reduction.cpp)
#target_link_libraries(${PROJECT_NAME}_camera_error_reduction ${catkin_LIBRARIES} lar_vision lar_tools)
#add_dependencies(${PROJECT_NAME}_camera_error_reduction ${PROJECT_NAME}_gencfg)


# height_map_segmentation_rf
#add_executable(${PROJECT_NAME}_height_map_segmentation_rf  src/nodes/height_map_segmentation_rf.cpp)
#target_link_libraries(${PROJECT_NAME}_height_map_segmentation_rf ${catkin_LIBRARIES} lar_vision lar_tools)

# TSDF INTEGRATION
#add_executable(${PROJECT_NAME}_tsdf_integration  src/nodes/tsdf_integration.cpp)
#target_link_libraries(${PROJECT_NAME}_tsdf_integration ${catkin_LIBRARIES} lar_vision lar_tools)

# TSDF ANALYZER
#add_executable(${PROJECT_NAME}_tsdf_analyzer  src/nodes/tsdf_analyzer.cpp)
#target_link_libraries(${PROJECT_NAME}_tsdf_analyzer ${catkin_LIBRARIES} lar_vision lar_tools)

#TESTING
# segmentation test
#add_executable(${PROJECT_NAME}_segmentation_test  src/nodes/testing/segmentation_test.cpp)
#target_link_libraries(${PROJECT_NAME}_segmentation_test ${catkin_LIBRARIES} lar_vision lar_tools)

#MARKER DETECTOR
add_executable(${PROJECT_NAME}_marker_detector  src/nodes/marker_detector.cpp)
target_link_libraries(${PROJECT_NAME}_marker_detector ${catkin_LIBRARIES} lar_vision lar_tools ${ARUCO_LIBS})

#PARMA DOUBLE SET
add_executable(${PROJECT_NAME}_parma_double_set  src/nodes/parma_double_set.cpp)
target_link_libraries(${PROJECT_NAME}_parma_double_set ${catkin_LIBRARIES} lar_vision lar_tools )
