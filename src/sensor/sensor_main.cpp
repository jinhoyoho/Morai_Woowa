#include "Morai_Woowa/sensor_main.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Sensor_main");

    ROS_INFO("Start Calibration, LiDAR_pre, Traffic...");

    LiDAR_pre lp;   // LiDAR_pre.cpp

    calibration cl; // calibration.cpp
    Traffic tf;     // traffic.cpp

    ros::spin();

    return 0;
}