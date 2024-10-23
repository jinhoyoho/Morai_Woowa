#ifndef ESCAPE_CONTROL_NODE_H
#define ESCAPE_CONTROL_NODE_H

#include <ros/ros.h>
#include <morai_msgs/SkidSteer6wUGVCtrlCmd.h>
#include <cmath>

class DynamicPlanning {
public:
    DynamicPlanning();
    void Brake();
    void Rear();
    void TurnLeft90();
    void TurnLeft180();
    void TurnRight180();

private:
    ros::NodeHandle nh_;
    ros::Publisher escape_ctrl_pub_;
};

#endif 
