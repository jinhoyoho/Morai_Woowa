#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Dense>  // Matrix

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

 // focal length
double fx = 320.0;
double fy = 320.0;

// principal points
double cx = 320.0;
double cy = 240.0;

// skew coefficient
double skew_c = 0;

// camera origin
double camera_x = 0.1;
double camera_y = 0.0;
double camera_z = 0.835;

// lidar origin
double lidar_x = 0.1;
double lidar_y = 0.0;
double lidar_z = 0.739;

// 카메라와 라이다의 각도(degree)
double camera_roll = 0.0;
double camera_pitch = 0.0;
double camera_yaw = 0.0;


class calibration
{
private:
    sensor_msgs::PointCloud2 lidar;
    ros::Subscriber image_sub;
    ros::Subscriber lidar_sub;
    ros::Publisher pub;
    
    cv::Mat frame;  // 이미지
    // std::vector<cv::Point3f> lidar_points;   // 라이다
    Eigen::MatrixXd lidar_points;   // 라이다
    
    Eigen::Matrix3d intrinsic;  // 카메라 내부 파라미터 3x3
    Eigen::Matrix<double, 3, 4> extrinsic;  // 외부 파라미터 3x4
    Eigen::Matrix<double, 3, 4> combined_matrix; // intrinsic x extrinsic

    Eigen::Vector3d camera_origin; // 카메라 원점
    Eigen::Vector3d lidar_origin; // 라이다 원점

    // Eigen::MatrixXd result; // 투영시킨 결과 좌표

    cv::Mat cameraMatrix;
    cv::Mat rvec;
    cv::Mat tvec;
    cv::Mat distCoeffs;



public:
    calibration();  // 생성자
    
    void do_cali();  // calibration 실행
    void image_callBack(const sensor_msgs::ImageConstPtr& msg); // 이미지 받기
    void lidar_callBack(const sensor_msgs::PointCloud2ConstPtr& msg); // Lidar 받기
    Eigen::Matrix3d computeRotationMatrix(double roll, double pitch, double yaw);   // 회전 행렬 구하기
    void projection(); // 라이다 점을 이미지에 투영

};