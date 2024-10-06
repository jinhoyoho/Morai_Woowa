#include <ros/ros.h>
#include "Morai_Woowa/obj_info.h"
#include <string>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>

class Traffic
{
    private:
        bool flag;  // 출발 플래그
        cv::Mat frame; // 이미지
        ros::Subscriber image_sub;
        ros::Subscriber object_sub;

    public:
        Traffic(); // 생성자

        void object_callBack(const Morai_Woowa::obj_info::ConstPtr& msg);
        void image_callBack(const sensor_msgs::ImageConstPtr& msg);
        void process_image(int xmin, int xmax, int ymin, int ymax);
};