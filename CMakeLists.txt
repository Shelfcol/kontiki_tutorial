cmake_minimum_required(VERSION 3.0.2)
project(kontiki_tutorial)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

set(ALL_TARGET_LIBRARIES "")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  # cv_bridge
  geometry_msgs
  # image_transport
  message_generation
  nav_msgs
  pcl_conversions
  roscpp
  roslib
  rospy
  sensor_msgs
  std_msgs
  tf
  # ndt_omp
  rosbag
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED)
# find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
# find_package(GTSAM REQUIRED QUIET)
find_package(Boost REQUIRED COMPONENTS filesystem program_options system)

set(PANGOLIN_DIR "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/build-pangolin")
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

list(APPEND ALL_TARGET_LIBRARIES ${Pangolin_LIBRARIES})

# include_directories("/home/gxf/multi-sensor-fusion/calib_ws/src/ad_sensor_fusion/src/lidar_imu_sync/third_party/Kontiki/include/entity/")
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/third_party/Kontiki)
include_directories(${PROJECT_SOURCE_DIR}/third_party/Kontiki/include) # important
list(APPEND third_party_lib  Kontiki)

set(HEADER_FILES  ${PROJECT_SOURCE_DIR}/include/kontiki_tutorial/trajectory_manager.hpp
                  ${PROJECT_SOURCE_DIR}/include/kontiki_tutorial/data_types.hpp
                  ${PROJECT_SOURCE_DIR}/include/kontiki_tutorial/kontiki_imu_test.hpp

)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
)
## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   nav_msgs#   sensor_msgs#   std_msgs
# )
include(cmake/global_definition.cmake)
include(cmake/yaml.cmake)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES kontiki_tutorial
#  CATKIN_DEPENDS cv_bridge geometry_msgs image_transport message_generation nav_msgs pcl_conversions roscpp roslib rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_definitions(${EIGEN3_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


# add_library(${PROJECT_NAME} src/lidar_imu_sync.cpp src/lidar_imu_calib.cpp)
add_executable(${PROJECT_NAME}_node src/main.cpp  ${HEADER_FILES})
# target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ndt_omp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${third_party_lib} ${ALL_TARGET_LIBRARIES})

add_executable(plotKittiTrajectory src/plotKittiTrajectory.cpp)
target_link_libraries(plotKittiTrajectory ${Pangolin_LIBRARIES} ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})