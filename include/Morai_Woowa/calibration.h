#include <iostream>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Quaternion.h>
#include <Eigen/Dense>  // Matrix
#include <vector>
#include <unordered_map>
#include <cmath>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "morai_woowa/obj_info.h"

#include <geometry_msgs/Vector3.h>  // Lidar 좌표를 publish


class calibration
{
private:
    ros::Subscriber lidar_sub;  // Lidar Pre를 받음
    ros::Subscriber object_sub; // detect한 이미지 사각형의 x, y와 이미지를 전달
    ros::Publisher lidar_pub;   // Lidar 좌표 publish
    
    cv::Mat frame;  // 이미지
    
    pcl::PointCloud<pcl::PointXYZI> cloud;   // pointcloud 저장

    std::vector<double> intensity;  // intensity 값 저장

    // box 크기
    int xmin;
    int xmax;
    int ymin;
    int ymax;
    
    std::vector<cv::Point3f> lidar_points;   // 라이다
    cv::Mat cameraMatrix;   // 카메라 내부 파라미터
    cv::Mat rvec;   // 회전 행렬
    cv::Mat tvec;   // 이동 벡터
    cv::Mat distCoeffs;     // 왜곡 벡터

    // 최소 좌표
    double min_x;  
    double min_y;
    double min_z;

    // 전역 변수로 하면 중복 선언으로 문제 발생

    // 각도를 라디안으로 변환하는 함수
    double deg2rad(double degrees) {
        return degrees * (M_PI / 180.0);
    }

    double fov = 60.0; // Field of View
    // focal length
    double fx = 640.0 / (2 * std::tan(deg2rad(fov/2)));
    double fy = fx;

    // principal points
    double cx = 320.0;
    double cy = 240.0;

    // skew coefficient
    double skew_c = 0;

    // camera origin
    double camera_x = 0.2;
    double camera_y = 0.0;
    double camera_z = 0.81;

    // lidar origin
    double lidar_x = 0.2;
    double lidar_y = 0.0;
    double lidar_z = 0.72;

    double min_distance;    // 최소 거리 갱신

    // timeStamp 맞추기
    ros::Time last_image_time;  // 마지막으로 받은 이미지 시간
    ros::Time last_lidar_time;  // 마지막으로 받은 라이다 시간

public:
    calibration(ros::NodeHandle& nh);  // 생성자
    
    void lidar_callBack(const sensor_msgs::PointCloud2ConstPtr& msg); // Lidar 받기
    void object_callBack(const morai_woowa::obj_info::ConstPtr& msg);
    void do_cali();  // calibration 실행
    void projection(cv::Mat frame); // 라이다 점을 이미지에 투영

    cv::Mat_<double> computeRotationMatrix(double roll, double pitch, double yaw);
};