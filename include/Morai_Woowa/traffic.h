#include <ros/ros.h>
#include "morai_woowa/obj_info.h"
#include <string>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <numeric> // std::accumulate

#define STOP 0
#define GO 1

class Traffic
{
    private:
        bool flag;  // 출발 플래그
        bool redFlag; // 빨간색을 보았는가
        cv::Mat frame; // 이미지
        ros::Subscriber image_sub;
        ros::Subscriber object_sub;
        std::vector<int> mvf; // 이동 평균 필터 10개만
        int count;  // 이동 평균 필터 개수

    public:
        Traffic(); // 생성자

        void object_callBack(const morai_woowa::obj_info::ConstPtr& msg);
        void image_callBack(const sensor_msgs::ImageConstPtr& msg);
        void process_image();
};