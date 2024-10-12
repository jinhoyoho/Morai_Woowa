#ifndef PATH_TRACKING_H
#define PATH_TRACKING_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include "Morai_Woowa/way_point.h"
#include <nav_msgs/Odometry.h>

typedef std::vector<Waypoint> path;

class PurePursuitController {
public:
    PurePursuitController(ros::NodeHandle& nh);
    // 경로를 받아서 처리하는 함수
    void publishPath(const nav_msgs::Path::ConstPtr& msg);
    // 현재 위치와 속도를 받아 Ego 상태를 업데이트
    void getRobotStatus(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);
    // odom 콜백 함수
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg); 
    // 스티어링 각도를 계산
    double steering_angle();
    double calculateCurvature();
    void controlLoop();


private:
    ros::Publisher ctrl_cmd_pub_;
    
    ros::Subscriber gpath_sub_;     // 경로 서브스크라이버
    ros::Subscriber current_pose_sub_; // 로봇 위치 서브스크라이버
    ros::Subscriber odom_sub_;

    path waypoint_path;
    Waypoint forward_point;
    geometry_msgs::Point current_position;
    
    double vehicle_yaw;
    double current_linear_vel;
    double current_angular_vel;
    double wheel_base;
    double lfd;
    double steering;
    bool is_look_forward_point;
    ros::Time previous_time;
    double previous_heading;
};

#endif 