## Dependency
```
1. ROS
2. sudo apt install libpcap-dev libyaml-cpp-dev
```

## Build
```
mkdir -p rosworkspace/src ; cd rosworkspace/src
git clone https://github.com/HesaiTechnology/Pandar40_ROS.git --recursive
cd ../
catkin_make
source ./devel/setup.sh
```

## Run
```
roslaunch hesai_lidar p40.launch
```

The driver will publish a PointCloud message in the topic.
```
/pandar_points
```

## Parameters:
```
	<arg name="server_ip" default="192.168.1.201"/>
	<arg name="lidar_recv_port"  default="2368"/>   lidar's port
	<arg name="gps_port"  default="10110"/>         gps's port
	<arg name="start_angle"  default="0"/>          lidar's start angle

  ......

	<param name="calibration_file" type="string" value="$(find hesai_lidar)/config/calibration.yml"/>  Calibration of Camera (Pandora Only, instrinsic and exstrinsic)
	<param name="lidar_correction_file"  type="string" value="$(find hesai_lidar)/config/correction.csv"/> Calibration of Lidar

```
