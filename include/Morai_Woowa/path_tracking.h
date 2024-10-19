// path_tracking.h
#ifndef PATH_TRACKING_H
#define PATH_TRACKING_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <cmath>
#include "Morai_Woowa/way_point.h"

class PurePursuitController {
public:
    PurePursuitController(ros::NodeHandle& nh);

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void getRobotStatus(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);
    void publishPath(const nav_msgs::Path::ConstPtr& msg);
    int findClosestWaypoint(double x, double y, int previous_index, const std::vector<Waypoint>& waypoints);
    double steering_angle();
    double calculateCurvature();
    void controlLoop();
    void Brake();
    void TurnLeft90();

private:
<<<<<<< HEAD
=======
    bool isWaypointBehind(double wx, double wy);
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
>>>>>>> e34b73addc2cb4d431b92c16cad1739c58554f87
    double wheel_base;
    double lfd;
    double steering;
    bool is_look_forward_point;
<<<<<<< HEAD
    double current_linear_vel;
    double current_angular_vel;
    double vehicle_yaw;
=======
    bool has_turned_left_90;
     bool at_goal;
>>>>>>> e34b73addc2cb4d431b92c16cad1739c58554f87
    ros::Time previous_time;
    double previous_heading;
    int current_waypoint_idx_;
    bool wpt_init_flag_;
    bool has_turned_left_;
    bool is_rotating_;
    double target_heading_; // 목표 헤딩

    enum ControllerState {
        FOLLOW_PATH,
        TURN_LEFT_90,
        AT_GOAL
    };

    ControllerState current_state_;

    std::vector<Waypoint> waypoint_path;
    geometry_msgs::Point current_position;
    Waypoint forward_point;
    ros::Publisher ctrl_cmd_pub_;
    ros::Subscriber gpath_sub_;
    ros::Subscriber current_pose_sub_;
    ros::Subscriber odom_sub_;

    // 상태별 함수
    void followPath(ros::Rate& rate);
    void turnLeft(ros::Rate& rate);
    bool isWaypointBehind(double wx, double wy);
};

<<<<<<< HEAD
#endif // PATH_TRACKING_H
=======
#endif 
>>>>>>> e34b73addc2cb4d431b92c16cad1739c58554f87
