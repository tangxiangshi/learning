cmake_minimum_required(VERSION 3.0.2)
project(SDC_XIANGSHI)


find_package(catkin REQUIRED COMPONENTS
roscpp
geometry_msgs
std_msgs
sensor_msgs
nav_msgs
rospy
tf
image_transport
visualization_msgs
cv_bridge
pcl_ros
pcl_conversions
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS EIGEN3 PCL
)



include_directories(

  include
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# target_link_libraries(ncrl ${catkin_LIBRARIES})
# add_dependencies(ncrl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(sdc src/sdc.cpp)
#記得target_link_libraries 要加ncrl
target_link_libraries(sdc ${catkin_LIBRARIES} ${PCL_INCLUDE_DIRS} ${CERES_LIBRARIES})
