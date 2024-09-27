#include "Morai_Woowa/calibration.h"

calibration::calibration()
{
    ros::NodeHandle nh;
    image_sub = nh.subscribe("python_image", 1, &calibration::image_callBack, this);
    pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
}
void calibration::image_callBack(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        // cv_bridge를 사용하여 ROS 이미지를 OpenCV 이미지로 변환
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        // 변환된 이미지를 보여줍니다.
        cv::imshow("Received Image", cv_ptr->image);
        if (cv::waitKey(10) == 27) exit(1); // ESC 키로 종료
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert image! %s", e.what());
        ROS_ERROR("Received image encoding: %s", msg->encoding.c_str());
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "calibration");

    calibration cl;

    ros::spin();
    return 0;
}