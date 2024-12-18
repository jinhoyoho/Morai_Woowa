#ifndef ESCAPE_CONTROL_NODE_H
#define ESCAPE_CONTROL_NODE_H

#include <ros/ros.h>
#include <morai_msgs/SkidSteer6wUGVCtrlCmd.h>
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <morai_woowa/ControlSrv.h>
#include <std_msgs/Bool.h>
#include <vector>

class DynamicPlanning {
public:
    DynamicPlanning();
    void waypointCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void traffic_callback(const std_msgs::Bool::ConstPtr& msg);
 
    void Brake();
    void Rear();

private:
    ros::NodeHandle nh_;
    ros::Publisher escape_ctrl_pub_;
    ros::Subscriber scurrent_pose_sub;
    ros::Subscriber traffic_sub_;

    double previous_position_x_;  // 이전 위치 x 좌표
    double previous_position_y_;  // 이전 위치 y 좌표
    ros::ServiceClient control_client_;
    ros::Time last_movement_time_;  // 마지막으로 움직였던 시간
    bool is_robot_stuck_;  // 로봇이 멈췄는지 여부
    double no_movement_duration_;  // 움직임이 없는 시간 기준

    bool traffic_go_;

    std::vector<std::vector<double>> pose_que;

    int hz = 30;
    int sec = 10;
    double dis = 1.0;
};

#endif 
