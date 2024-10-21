#include "Morai_Woowa/sensor_main.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Sensor_main");
    ros::NodeHandle nh;

    ROS_INFO("Start Calibration, LiDAR_pre, Traffic...");

    LiDAR_pre lp(nh);   // LiDAR_pre.cpp

    //calibration cl(nh); // calibration.cpp
    // calibration2 cl2(nh); // calibration.cpp
    calibration3 cl3(nh); // calibration.cpp
    Traffic tf(nh);     // traffic.cpp

    ros::spin();

    return 0;
}