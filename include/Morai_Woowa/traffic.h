#include <ros/ros.h>
#include "morai_woowa/obj_info.h"
#include "morai_woowa/traffic_srv.h"
#include <std_msgs/Bool.h>
#include <string>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <numeric> // std::accumulate
#include <geometry_msgs/PoseStamped.h>



#define STOP 0
#define GO 1

class Traffic
{
    private:
        bool flag;  // 출발 플래그
        bool redFlag; // 빨간색을 보았는가
        bool crosswalk; // 횡단보도인지 아닌지 -> request로 받음
        cv::Mat frame; // 이미지
        ros::Subscriber image_sub;
        ros::Subscriber pose_sub;
        ros::Publisher flag_pub;
        std::vector<int> mvf; // 이동 평균 필터 10개만
        int count;  // 이동 평균 필터 개수
        float current_x;
        float current_y;
        bool first_false_flag;
        ros::Time first_false_time;

    public:
        Traffic(ros::NodeHandle& nh); // 생성자

        void object_callBack(const morai_woowa::obj_info::ConstPtr& msg);
        void image_callBack(const sensor_msgs::ImageConstPtr& msg);
        void process_image();
        void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
};