#include "Morai_Woowa/person_action_node.h"

person_action_node::person_action_node(ros::NodeHandle& nh):PCAserver_(nh, "person_collision_action", boost::bind(&person_action_node::execute, this, _1), false)
{
    PCAserver_.start(); // 액션 서버 시작
    lidar_coord_sub = nh.subscribe("lidar_coord", 10, &person_action_node::coord_callBack, this);
    current_pose_sub_ = nh.subscribe("/current_pose", 10, &person_action_node::currentPoseCallback, this);

    ctrl_cmd_pub_ = nh.advertise<morai_msgs::SkidSteer6wUGVCtrlCmd>("/6wheel_skid_ctrl_cmd", 10);
    //초기 range
    person_range_ = 5.0;
    closest_person_.x = 0;
    closest_person_.y = 0;

    current_x_ = 0;
    current_y_ = 0;

    last_target_found_time_ = ros::Time::now();
    double averageDistance = 9999999;
}

bool person_action_node::check_collision_success(){

    float dis = 10000;
    
    Spot respawn_spot_indoor(430.00, -140.00);// 실내 리스폰 지점
    dis = calculateDistance(current_x_, current_y_, respawn_spot_indoor.x, respawn_spot_indoor.y);        

    if (dis < 0.5){
        ROS_WARN("indoor teleport success");
        return true;
    }

    Spot respawn_spot_outdoor(430.00, -140.00);// 야외 리스폰 지점
    dis = calculateDistance(current_x_, current_y_, respawn_spot_outdoor.x, respawn_spot_outdoor.y);        

    if (dis < 0.5){
        ROS_WARN("outdoor teleport success");
        return true;
    }

    else{
        ROS_WARN("teleport fail");
        return false;
    }
}

void person_action_node::execute(const morai_woowa::Person_Collision_ActGoalConstPtr& goal)
{
    ROS_INFO("Execute action: Person Collision Action!");

    //파라미터 초기화
    is_target = false;
    last_target_found_time_ = ros::Time::now();

    person_range_ = goal->range;

    morai_woowa::Person_Collision_ActFeedback feedback;
    morai_woowa::Person_Collision_ActResult result;

    while(ros::ok){

        ros::spinOnce();

        feedback.is_target = is_target;
        
        PCAserver_.publishFeedback(feedback);

        if (check_collision_success()){
            result.success = true;
            break;
        }
        if ((ros::Time::now() - last_target_found_time_).toSec() > 10.0)
        {
            result.success = false;
            ROS_WARN("No target detected for more than 10 seconds. Exiting loop.");
            break;
        }

        if(!is_target){
            continue;
        }   

        float ang_vel;  
        float lin_vel;

        double angle_rad = atan2(closest_person_.y, closest_person_.x);
        
        ang_vel = angle_rad > 0 ? ANG_VEL : -ANG_VEL;
        lin_vel = LIN_VEL;
        
        morai_msgs::SkidSteer6wUGVCtrlCmd ctrl_cmd;
        ctrl_cmd.cmd_type = 3;
        ctrl_cmd.Target_linear_velocity = lin_vel;
        ctrl_cmd.Target_angular_velocity = ang_vel;
        ctrl_cmd_pub_.publish(ctrl_cmd);

    }

    PCAserver_.setSucceeded(result);
}

void person_action_node::coord_callBack(const geometry_msgs::Vector3::ConstPtr& msg)
{
    ROS_INFO("Recived Lidar Coord: %f %f %f", msg->x, msg->y, msg->z);
    // 거리 계산
    averageDistance = std::sqrt(msg->x * msg->x + 
                                        msg->y * msg->y + 
                                        msg->z * msg->z);
    ROS_INFO("Distance: %f", averageDistance);

    is_target = false;

    if(averageDistance < person_range_){
        closest_person_.x = msg->x;
        closest_person_.y = msg->y;

        is_target = true;
        last_target_found_time_ = ros::Time::now();
    }

}

void person_action_node::currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_x_ = msg->pose.position.x;
    current_y_ = msg->pose.position.y;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "person_collision_action_node");
    ros::NodeHandle nh;

    person_action_node PAN(nh);

    ros::spin();
    return 0;
}