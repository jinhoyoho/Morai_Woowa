#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Dense>  // Matrix

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

#include "Morai_Woowa/obj_info.h"
#include "Morai_Woowa/LiDAR_pre.h"
#include "Morai_Woowa/traffic.h"


 // focal length
double fx = 320.0;
double fy = 320.0;

// principal points
double cx = 320.0;
double cy = 240.0;

// skew coefficient
double skew_c = 0;


class calibration
{
private:
    sensor_msgs::PointCloud2 lidar;
    ros::Subscriber image_sub;
    ros::Subscriber lidar_sub;
    ros::Subscriber object_sub;
    ros::Publisher pub;
    
    cv::Mat frame;  // 이미지
    Eigen::MatrixXd lidar_points;   // 라이다
    
    Eigen::Matrix3d intrinsic;  // 카메라 내부 파라미터 3x3
    Eigen::Matrix<double, 3, 4> extrinsic;  // 외부 파라미터 3x4



public:
    calibration();  // 생성자
    
    void image_callBack(const sensor_msgs::ImageConstPtr& msg); // 이미지 받기
    void lidar_callBack(const sensor_msgs::PointCloud2ConstPtr& msg); // Lidar 받기
    void object_callBack(const Morai_Woowa::obj_info::ConstPtr& msg);
    void do_cali();  // calibration 실행
    void projection(); // 라이다 점을 이미지에 투영

};