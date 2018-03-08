## Build
```
catkin_make
```

## Run
### Pandar40
```
roslaunch hesai_lidar p40.launch
```

### Pandar40P
```
roslaunch hesai_lidar p40p.launch
```

### Pandora
```
roslaunch hesai_lidar pandora_ros.launch
```

There is 6 node of Hesai Lidar ROS
```
/pandar_points
/hesai_lidar/pandora_camera0 (Only Pandora)
/hesai_lidar/pandora_camera1 (Only Pandora)
/hesai_lidar/pandora_camera2 (Only Pandora)
/hesai_lidar/pandora_camera3 (Only Pandora)
/hesai_lidar/pandora_camera4 (Only Pandora)
```

## Parameters:
```
	<arg name="server_ip" default="192.168.20.51"/> pandora's ip
	<arg name="server_port"  default="9870"/>       pandora's camera port
	<arg name="lidra_recv_port"  default="8080"/>   lidar's port
	<arg name="gps_port"  default="10110"/>         gps's port
	<arg name="start_angle"  default="0"/>          lidar's start angle

  ......

	<param name="calbration_file" type="string" value="$(find hesai_lidar)/config/calibration.yml"/>  Calibration of Camera (Pandora Only, instrinsic and exstrinsic)
	<param name="lidar_correction_file"  type="string" value="$(find hesai_lidar)/config/correction.csv"/> Calibration of Lidar

```
