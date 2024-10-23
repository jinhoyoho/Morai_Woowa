#include <iostream>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Quaternion.h>
#include <Eigen/Dense>  // Matrix
#include <vector>
#include <unordered_map>
#include <cmath>
#include <deque>  
#include <algorithm> 

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "morai_woowa/obj_info2.h"
#include "morai_woowa/obj_array3.h"
#include "morai_woowa/average_points.h"
#include "morai_woowa/average_points_array.h"

#include <geometry_msgs/Vector3.h>  // Lidar 좌표를 publish


// BoundingBox 구조체 정의
struct BoundingBox {
    int16_t xmin;
    int16_t xmax;
    int16_t ymin;
    int16_t ymax;
};

class calibration3
{
private:
    ros::Subscriber lidar_sub;  // Lidar Pre를 받음
    ros::Subscriber object_sub1; // detect한 이미지 사각형의 x, y와 이미지를 전달
    ros::Subscriber object_sub2; // detect한 이미지 사각형의 x, y와 이미지를 전달
    ros::Publisher lidar_pub;   // Lidar 좌표 publish
    ros::Publisher points_array_pub;
    
    cv::Mat frame;  // 이미지
    
    pcl::PointCloud<pcl::PointXYZI> cloud;   // pointcloud 저장

    std::vector<double> intensity;  // intensity 값 저장

    // box 크기
    int xmin;
    int xmax;
    int ymin;
    int ymax;
    
    std::map<int, std::vector<cv::Point3f>> lidar_points;   // 라이다
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

    double fov = 75.0; // Field of View
    // focal length
    double fx = 640.0 / (2 * std::tan(deg2rad(fov/2)));
    double fy = fx;

    // principal points
    double cx = 320.0;
    double cy = 240.0;

    // skew coefficient
    double skew_c = 0;

    // camera origin
    double camera_x_1 = 0.250;
    double camera_y_1 = -0.150;
    double camera_z_1 = 0.730;  // yaw: -

    double camera_x_2 = 0.250;
    double camera_y_2 = 0.150;
    double camera_z_2 = 0.730;  // yaw:: +

    // lidar origin
    double lidar_x = 0.2;
    double lidar_y = 0.0;
    double lidar_z = 0.72;

    double min_distance;    // 최소 거리 갱신

    // timeStamp 맞추기
    ros::Time last_image_time;  // 마지막으로 받은 이미지 시간
    ros::Time last_lidar_time;  // 마지막으로 받은 라이다 시간

    std::deque<morai_woowa::obj_array3::ConstPtr> img_msg_queue; // msg 큐
    std::deque<morai_woowa::obj_array3::ConstPtr> img_msg_queue2; // msg 큐


public:
    calibration3(ros::NodeHandle& nh);  // 생성자
    
    void lidar_callBack(const sensor_msgs::PointCloud2ConstPtr& msg); // Lidar 받기
    void object_callBack1(const morai_woowa::obj_array3::ConstPtr& msg);
    void object_callBack2(const morai_woowa::obj_array3::ConstPtr& msg);
    void do_cali();  // calibration 실행
    void projection(cv::Mat frame, std::vector<BoundingBox> bounding_boxes, int type); // 라이다 점을 이미지에 투영

    cv::Mat_<double> computeRotationMatrix(double roll, double pitch, double yaw);
};