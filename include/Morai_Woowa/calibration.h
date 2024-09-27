#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Dense>  // Matrix

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>


using Eigen::MatrixXd;

class calibration
{
private:
    sensor_msgs::PointCloud2 lidar;
    ros::Subscriber image_sub;
    ros::Publisher pub;
    cv::Mat frame;

public:
    calibration();
    void image_callBack(const sensor_msgs::ImageConstPtr& msg);
};