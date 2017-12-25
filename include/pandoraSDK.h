#ifndef __PANDORA_SDK_H
#define __PANDORA_SDK_H

#include <string>
#include <boost/function.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "point_types.h"

class PandoraSDK_internal;

class PandoraSDK
{
public:
	PandoraSDK(
		const std::string pandoraIP, 														// pandora的ip
		const unsigned short pandoraCameraPort,		   						// pandora的port
		const unsigned short lidarRecvPort,     								// lidar的数据接收端口， 默认为8080
		const unsigned short gpsRecvPort,												// gps数据接收端口， 默认为10110
		const double startAngle,		  													// lidar的旋转起始角，默认为0,单位是度，例如135度
		const std::string intrinsicFile, 												// 摄像头的内参文件路径，为空时，输出的数据是未经过矫正的
		const std::string lidarCorrectionFile, 									// lidar的标定文件路径，为空时，将使用默认参数
		boost::function<void(boost::shared_ptr<cv::Mat> matp, double timestamp, int pic_id)> cameraCallback, 	// 摄像头的callback函数
		boost::function<void(boost::shared_ptr<PPointCloud> pcloudp, double timestamp)> lidarCallback, 				// lidar的callback函数
		boost::function<void(double timestamp)> gpsCallback);		// gps数据的callback函数
	
	PandoraSDK(
		const std::string pandoraIP,
		const unsigned short pandoraCameraPort,
		boost::function<void(boost::shared_ptr<cv::Mat> matp, double timestamp, int pic_id)> cameraCallback,
		boost::function<void(boost::shared_ptr<PPointCloud> pcloudp, double timestamp)> lidarCallback);

	PandoraSDK(
		const std::string pcapPath,
		const std::string lidarCorrectionFile,
		const double startAngle,
		boost::function<void(boost::shared_ptr<PPointCloud> pcloudp, double timestamp)> lidarCallback);

	PandoraSDK(){};

	~PandoraSDK();
	int start();
	void stop();
private:
	PandoraSDK_internal *psi;
};

#endif
