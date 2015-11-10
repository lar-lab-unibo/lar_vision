cmake_minimum_required(VERSION 2.8.3)
project(lar_vision)

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
find_package(PCL 1.8 REQUIRED  )
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


#Lar Tools
MESSAGE("#### LAR TOOLS #####")
find_package(lar_tools)
include_directories(${lar_tools_INCLUDE_DIRS})


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lar_vision
#  CATKIN_DEPENDS cv_bridge image_transport pcl_ros roscpp rospy
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  include/${PROJECT_NAME}/
  include/${PROJECT_NAME}/commons
  include/${PROJECT_NAME}/segmentation
)


#LAR VISION COMMONS LIBRARY
add_library(lar_vision_commons
        src/commons/lar_vision_commons.cpp
        src/segmentation/HighMap.cpp
)
target_link_libraries (lar_vision_commons ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} lar_tools)



#NODES

# TESTING NODE
add_executable(${PROJECT_NAME}_testing_node  src/nodes/testing_node.cpp)
target_link_libraries(${PROJECT_NAME}_testing_node ${catkin_LIBRARIES} lar_vision_commons lar_tools)


