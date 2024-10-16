#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "morai_woowa/Person_Collision_ActAction.h"
#include <geometry_msgs/Vector3.h>

class person_action_node
{
private:
    ros::Subscriber lidar_coord_sub;    // 라이다 좌표
    
    actionlib::SimpleActionServer<morai_woowa::Person_Collision_ActAction> PCAserver_;
    morai_woowa::Person_Collision_ActFeedback feedback_;
    morai_woowa::Person_Collision_ActResult result_;

public:
    person_action_node(ros::NodeHandle& nh);

    void coord_callBack(const geometry_msgs::Vector3::ConstPtr& msg);    // 좌표
    void execute(const morai_woowa::Person_Collision_ActGoalConstPtr& goal);
};