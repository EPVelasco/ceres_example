cmake_minimum_required(VERSION 3.0.2)
project(ceres_example)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  geometry_msgs
  nav_msgs
  rosbag
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  
)
include_directories( "/usr/include/dqrobotics" )

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)


include_directories(
  include
  ${catkin_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

link_directories(
  include
  ${PCL_LIBRARY_DIRS}
  ${CERES_LIBRARY_DIRS}
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
  DEPENDS EIGEN3 PCL Ceres 
  INCLUDE_DIRS include
)

#add_executable(opti_node src/opti_ceres.cpp)
#target_link_libraries(opti_node ${EIGEN3_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(bunny_node src/bunny.cpp)
target_link_libraries(bunny_node ${EIGEN3_LIBRARIES} ${catkin_LIBRARIES} ${CERES_LIBRARIES})

add_executable(udq_bunny_node src/bunny_UDQ.cpp)
target_link_libraries(udq_bunny_node ${EIGEN3_LIBRARIES} ${catkin_LIBRARIES} ${CERES_LIBRARIES})


add_executable(pcl_map_node src/pcl_map_diff.cpp)
target_link_libraries(pcl_map_node ${EIGEN3_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})
