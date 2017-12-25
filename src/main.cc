#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include "pandoraSDK.h"

class PandoraClient
{
public:
  PandoraClient(ros::NodeHandle node, ros::NodeHandle nh)
  {
    lidarPublisher = node.advertise<sensor_msgs::PointCloud2>("pandar_points", 10);

    image_transport::ImageTransport it(nh);
    imgPublishers[0] = it.advertise("pandora/pandora_camera0", 1);
    imgPublishers[1] = it.advertise("pandora/pandora_camera1", 1);
    imgPublishers[2] = it.advertise("pandora/pandora_camera2", 1);
    imgPublishers[3] = it.advertise("pandora/pandora_camera3", 1);
    imgPublishers[4] = it.advertise("pandora/pandora_camera4", 1);

    std::string serverIp;
    int serverPort;
    std::string calibrationFile;
    int lidarRecvPort;
    int gpsPort;
    double startAngle;
    std::string lidarCorrectionFile;
    
    nh.getParam("server_ip", serverIp);
    nh.getParam("server_port", serverPort);
    nh.getParam("calbration_file", calibrationFile);
    nh.getParam("lidra_recv_port", lidarRecvPort);
    nh.getParam("gps_port", gpsPort);
    nh.getParam("start_angle", startAngle);
    nh.getParam("lidar_correction_file", lidarCorrectionFile);

    psdk = PandoraSDK(serverIp, serverPort, lidarRecvPort, gpsPort, startAngle,
                      calibrationFile,
                      lidarCorrectionFile,
                      boost::bind(&PandoraClient::cameraCallback, this, _1, _2, _3),
                      boost::bind(&PandoraClient::lidarCallback, this, _1, _2),
                      NULL);
    psdk.start();
  }

  void cameraCallback(boost::shared_ptr<cv::Mat> matp, double timestamp, int pic_id)
  {

    sensor_msgs::ImagePtr imgMsg;

    switch (pic_id)
    {
    case 0:
      imgMsg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", *matp).toImageMsg();
      break;
    case 1:
    case 2:
    case 3:
    case 4:
      imgMsg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", *matp).toImageMsg();
      break;
    default:
      ROS_INFO("picid wrong in getImageToPub");
      return;
    }
    imgMsg->header.stamp = ros::Time(timestamp);
    imgPublishers[pic_id].publish(imgMsg);

  }

  void lidarCallback(boost::shared_ptr<PPointCloud> cld, double timestamp)
  {
    pcl_conversions::toPCL(ros::Time(timestamp), cld->header.stamp);
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cld, output);
    lidarPublisher.publish(output);
  }


private:
  ros::Publisher lidarPublisher;
  image_transport::Publisher imgPublishers[5];
  PandoraSDK psdk;
};




int main(int argc, char **argv)
{
  ros::init(argc, argv, "pandora_ros");
  ros::NodeHandle nh("~");
  ros::NodeHandle node;
  PandoraClient pandoraClient(node, nh);

  ros::spin();
  return 0;
}