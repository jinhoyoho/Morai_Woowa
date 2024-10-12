#ifndef DYNAMIC_PLANNING_H
#define DYNAMIC_PLANNING_H

#include <ros/ros.h>
#include <morai_msgs/SkidSteer6wUGVCtrlCmd.h>

class DynamicPlanning {
public:
    DynamicPlanning();
    void Brake();
    void Rear();
    void TurnRight90();
    void TurnLeft90();
    void Turn180();

private:
    ros::NodeHandle nh_;
    ros::Publisher ctrl_cmd_pub_;
};

#endif 
