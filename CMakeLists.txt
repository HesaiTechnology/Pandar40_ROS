cmake_minimum_required(VERSION 2.8.3)
project(hesai_lidar)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)



find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  sensor_msgs
)
find_package( Boost REQUIRED )
find_package(PCL REQUIRED )



catkin_package()

add_subdirectory(src/HesaiLidarSDK)
add_executable(hesai_lidar_node 
	src/main.cc
)

add_definitions("-DHESAI_WITH_CAMERA")
include_directories(
  src/HesaiLidarSDK/include
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)



target_link_libraries(hesai_lidar_node 
	${catkin_LIBRARIES}
  	hesaiLidarSDK
)
