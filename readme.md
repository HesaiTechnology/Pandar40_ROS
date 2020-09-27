# Pandar40_ROS

## About the project
Pandar40_ROS project includes the ROS Driver for **Pandar40** LiDAR sensor manufactured by Hesai Technology.   
Note: For the driver of Pandar40P and other modules, please go to [HesaiLidar_General_ROS](https://github.com/HesaiTechnology/HesaiLidar_General_ROS)

Developed based on [Pandar40_SDK](https://github.com/HesaiTechnology/Pandar40_SDK), after launched, the project will monitor UDP packets from Pandar40 Lidar, parse data and publish point cloud frames into ROS under topic: ```/pandar```. It can also be used as an official demo showing how to work with Pandar40_SDK.

## Environment and Dependencies
**System environment requirement: Linux + ROS**  

　Recommanded:  
　Ubuntu 16.04 - with ROS kinetic desktop-full installed or  
　Ubuntu 18.04 - with ROS melodic desktop-full installed  
　Check resources on http://ros.org for installation guide 
 
**Library Dependencies: libpcap-dev + libyaml-cpp-dev**  
```
$sudo apt install libpcap-dev libyaml-cpp-dev
```

## Download and Build

**Install `catkin_tools`**
```
$ sudo apt-get update
$ sudo apt-get install python-catkin-tools
```
**Download code**  
```
$ mkdir -p rosworkspace/src
$ cd rosworkspace/src
$ git clone https://github.com/HesaiTechnology/Pandar40_ROS.git --recursive
```
**Build**
```
$ cd ..
$ catkin_make
```

## Configuration 
```
 $ gedit src/launch/p40.launch
```
**Reciving data from connected Lidar: config lidar ip&port, leave the pcap_file empty**
|Parameter | Default Value|
|---------|---------------|
|server_ip |192.168.1.201|
|lidar_recv_port |2368|
|gps_recv_port  |10110|
|laser_return_type| 0 |
|pcap_file ||　　

Data source will be from connected Lidar when "pcap_file" set to empty  
Make sure the parameters above set to the same with Lidar setting  
Note: (laser_return_type: for single return set to 1, for dural return set to 1)  

**Reciving data from pcap file: config pcap_file and correction file path**
|Parameter | Value|
|---------|---------------|
|pcap_file |pcap file path|
|lidar_correction_file |lidar correction file path|
|laser_return_type| return type of lidar in pcap file |

Data source will be from pcap file once "pcap_file" not empty 


## Run

1. Make sure current path in the `rosworkspace` directory
```
$ source devel/setup.bash
$ roslaunch hesai_lidar p40.launch
```
2. The driver will publish PointCloud messages to the topic `/pandar_points`  
3. Open Rviz and add display by topic  
4. Change fixed frame to `pandar` to view published point clouds  
