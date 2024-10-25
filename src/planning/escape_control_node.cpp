#include "Morai_Woowa/escape_control_node.h"
#include <geometry_msgs/PoseStamped.h>
#include <limits>
#include <ros/ros.h>

DynamicPlanning::DynamicPlanning() : nh_("~"), is_robot_stuck_(true), no_movement_duration_(15.0) {
    escape_ctrl_pub_ = nh_.advertise<morai_msgs::SkidSteer6wUGVCtrlCmd>("/escape_ctrl", 10);//escape_ctrl
    scurrent_pose_sub = nh_.subscribe("/current_pose", 10, &DynamicPlanning::waypointCallback, this);
    control_client_ = nh_.serviceClient<morai_woowa::ControlSrv>("/Control_srv");
    traffic_sub_ = nh_.subscribe("/traffic", 10, &DynamicPlanning::traffic_callback, this);

    traffic_go_ = true;

    // 이전 위치 초기화
    previous_position_x_ = std::numeric_limits<double>::max();
    previous_position_y_ = std::numeric_limits<double>::max();
    last_movement_time_ = ros::Time::now();

    pose_que.reserve(hz*sec);
}

void DynamicPlanning::traffic_callback(const std_msgs::Bool::ConstPtr& msg) {
    traffic_go_ = msg->data;
}

void DynamicPlanning::waypointCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    pose_que.push_back({msg->pose.position.x, msg->pose.position.y});
    
    if(pose_que.size() > 150){
        pose_que.erase(pose_que.begin());
    }

    if(pose_que.size() == 150){
        for(int i=0; i<pose_que.size(); i++){
            auto current_x = pose_que.back()[0];
            auto current_y = pose_que.back()[1];
            
            auto square_distance = (current_x - pose_que[i][0])*(current_x - pose_que[i][0]) + (current_y- pose_que[i][1])*(current_x - pose_que[i][1]);
            if(square_distance > dis*dis){
                is_robot_stuck_ = false;
                break;
            }
        }
    }


    if(pose_que.size() > 149 && is_robot_stuck_ && traffic_go_){
        std::cout<<"goback!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<"\n";
        Rear();
    }

    // if(!traffic_go_){
    //     last_movement_time_ = ros::Time::now();
    //     return;
    // }
    
    // double current_position_x = msg->pose.position.x;
    // double current_position_y = msg->pose.position.y;

    // // 로봇이 움직이지 않았는지 확인 (위치가 거의 변하지 않았을 경우)
    // double distance_moved = hypot(current_position_x - previous_position_x_, current_position_y - previous_position_y_);
    // // 로봇이 멈췄고, 멈춘 시간이 7초 이상일 때 후진 명령 실행
    // if (distance_moved < 1) {  // 이동이 거의 없을 때
    //     if ((ros::Time::now() - last_movement_time_).toSec() > no_movement_duration_ && !is_robot_stuck_) {
    //         ROS_WARN("Robot seems to be stuck for 7 seconds. Executing Rear function...");
    //         morai_woowa::ControlSrv control_srv;
    //         control_srv.request.mode = 3;  // mode 3로 전환 요청
    
    //         if (control_client_.call(control_srv)) {
    //             ROS_INFO("Successfully switched back to mode 3 (escape mode)");
    //         } else {
    //             ROS_ERROR("Failed to switch back to mode 3");
    //         }
            
    //         Rear();  
    //         is_robot_stuck_ = true;  // 후진 실행 후 상태 업데이트
    //     }
    // } else {  // 로봇이 움직이면 last_movement_time을 갱신하고 stuck 상태 해제
    //     last_movement_time_ = ros::Time::now();  // 마지막으로 움직인 시간을 갱신
    //     is_robot_stuck_ = false;  // 로봇이 움직였으므로 stuck 상태 해제
    // }

    // // 이전 위치를 현재 위치로 업데이트
    // previous_position_x_ = current_position_x;
    // previous_position_y_ = current_position_y;
}

void DynamicPlanning::Brake() {
    morai_msgs::SkidSteer6wUGVCtrlCmd escape_ctrl;
    escape_ctrl.cmd_type = 3;
    escape_ctrl.Target_linear_velocity = 0;
    escape_ctrl.Target_angular_velocity = 0;
    escape_ctrl_pub_.publish(escape_ctrl);
    ROS_INFO("Brake Activated!");
}

void DynamicPlanning::Rear() {
    morai_msgs::SkidSteer6wUGVCtrlCmd escape_ctrl;
    escape_ctrl.cmd_type = 3;
    escape_ctrl.Target_linear_velocity = -2.0;  // 후진 속도 -2 m/s
    escape_ctrl.Target_angular_velocity = 0;

    double rear_time = 2.5;  // 5m 후진
    ros::Time start_time = ros::Time::now();

    morai_woowa::ControlSrv control_srv;
    control_srv.request.mode = 3;  // mode 3으로 전환 요청
    
    if (control_client_.call(control_srv)) {
        ROS_INFO("Successfully switched to mode 3 ");
    } else {
        ROS_ERROR("Failed to switch to mode 3");
    }

    ros::Rate rate(10); 
    while (ros::ok() && (ros::Time::now() - start_time).toSec() < rear_time) {
        escape_ctrl_pub_.publish(escape_ctrl); 
        rate.sleep();
    }

    Brake(); 
    ROS_INFO("Moved 5m Backwards and Stopped!");
    // Control mode를 1로 변경하는 서비스 호출

    // morai_woowa::ControlSrv control_srv;
    control_srv.request.mode = 0;  // mode 3으로 전환 요청
    
    if (control_client_.call(control_srv)) {
        ROS_INFO("Successfully switched back mode 0");
    } else {
        ROS_ERROR("Failed to switch back to mode 0");
    }


}


int main(int argc, char** argv) {
    ros::init(argc, argv, "escape_ctrl_node");
    ros::NodeHandle nh;

    DynamicPlanning dp;

    ros::Rate loop_rate(10); 
    ros::spin();
    return 0;
}