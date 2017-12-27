# pandora-ros使用说明：
**注意:仅支持64位ubuntu16.04和64位ubuntu14.04**
## (1) 运行方法:
cd到项目目录,然后执行以下命令运行:
```
mkdir build
cd build
cmake ..
make -j4
source devel/setup.sh
roslaunch pandora_ros pandora_ros.launch
```
运行成功后将有6个topic:
```
/pandar_points
/pandora_ros/pandora_camera0
/pandora_ros/pandora_camera1
/pandora_ros/pandora_camera2
/pandora_ros/pandora_camera3
/pandora_ros/pandora_camera4
```

## (2)参数说明:
运行时可设置的参数可在lanuch/pandora_ros.lanuch中查看:
```
	<arg name="server_ip" default="192.168.20.51"/> pandora的ip
	<arg name="server_port"  default="9870"/>       pandora的tcp连接端口
	<arg name="lidra_recv_port"  default="8080"/>   lidar数据的接收端口
	<arg name="gps_port"  default="10110"/>         gps数据的接收端口
	<arg name="start_angle"  default="0"/>          lidar起始旋转角度

  ......

	<param name="calbration_file" type="string" value="$(find pandora_ros)/config/calibration.yml"/>  摄像头的矫正文件路径,文件不存在时输出的图像是未经矫正的
	<param name="lidar_correction_file"  type="string" value="$(find pandora_ros)/config/correction.csv"/> lidar的矫正文件路径,文件不存在时,使用默认的矫正参数

```
例如:
```
roslaunch pandora_ros pandora_ros.launch server_ip:=172.32.2.111
```
表示所要连接的pandora的ip是172.32.2.111