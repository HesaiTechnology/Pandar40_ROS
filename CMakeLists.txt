cmake_minimum_required(VERSION 2.8.3)
project(pandora_ros)

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

add_executable(pandora_ros_node 
	src/main.cc
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)


target_link_libraries(pandora_ros_node 
	${catkin_LIBRARIES}
  pandoraSDK
)
