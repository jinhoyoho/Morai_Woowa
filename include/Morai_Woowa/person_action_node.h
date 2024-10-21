#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "morai_woowa/Person_Collision_ActAction.h"
#include <morai_msgs/SkidSteer6wUGVCtrlCmd.h>
#include <morai_woowa/average_points_array.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>

#include <cmath>

float LIN_VEL(7.2);

struct person{
    float x;
    float y;
    int index;
}; 
struct Spot {
    float x;
    float y;
    Spot(float x_coord, float y_coord) : x(x_coord), y(y_coord) {}
};


class person_action_node
{
private:
    ros::Subscriber lidar_coord_sub;    // 라이다 좌표
    ros::Publisher ctrl_cmd_pub_; 
    ros::Subscriber current_pose_sub_;

    
    actionlib::SimpleActionServer<morai_woowa::Person_Collision_ActAction> PCAserver_;

    person closest_person_;
    float person_range_;
    bool is_target;
    double averageDistance;
    ros::Time last_target_found_time_;

    float current_x_;
    float current_y_;

public:
    person_action_node(ros::NodeHandle& nh);

    void coord_callBack(const morai_woowa::average_points_array::ConstPtr& msg);    
    void execute(const morai_woowa::Person_Collision_ActGoalConstPtr& goal);
    void currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg); 
    //void currentPoseCallback(const morai_woowa::average_points_array::ConstPtr& msg); 
    bool check_collision_success();

    double calculateDistance(double x1, double y1, double x2, double y2) {
        return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
    }
};