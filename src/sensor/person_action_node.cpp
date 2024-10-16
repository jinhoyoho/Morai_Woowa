#include "Morai_Woowa/person_action_node.h"

person_action_node::person_action_node(ros::NodeHandle& nh):PCAserver_(nh, "person_collision_action", boost::bind(&person_action_node::execute, this, _1), false)
{
    PCAserver_.start(); // 액션 서버 시작
    lidar_coord_sub = nh.subscribe("lidar_coord", 10, &person_action_node::coord_callBack, this);
}


void person_action_node::execute(const morai_woowa::Person_Collision_ActGoalConstPtr& goal)
{
    ROS_INFO("Execute action: Person Collision Action!");
    // 모두 실패!
    // feedback_.is_target = false;
    // PCAserver_.publishFeedback(feedback_);

}


void person_action_node::coord_callBack(const geometry_msgs::Vector3::ConstPtr& msg)
{
    ROS_INFO("Recived Lidar Coord: %f %f %f", msg->x, msg->y, msg->z);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "person_collision_action_node");
    ros::NodeHandle nh;

    person_action_node PAN(nh);

    ros::spin();
    return 0;
}