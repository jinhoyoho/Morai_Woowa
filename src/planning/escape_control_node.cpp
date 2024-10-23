#include "Morai_Woowa/escape_control_node.h"

DynamicPlanning::DynamicPlanning() : nh_("~") {
    escape_ctrl_pub_ = nh_.advertise<morai_msgs::SkidSteer6wUGVCtrlCmd>("/escape_ctrl", 10);//escape_ctrl
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
    escape_ctrl.Target_linear_velocity = -2;  // 후진 속도 -2 m/s
    escape_ctrl.Target_angular_velocity = 0;

    double rear_time = 2.5;  // 5m 후진
    ros::Time start_time = ros::Time::now();

    ros::Rate rate(10); 
    while (ros::ok() && (ros::Time::now() - start_time).toSec() < rear_time) {
        escape_ctrl_pub_.publish(escape_ctrl);  // 후진 명령 퍼블리시
        rate.sleep();
    }

    Brake();  // 후진 후 정지
    ROS_INFO("Moved 5m Backwards and Stopped!");

}

void DynamicPlanning::TurnLeft90() {
    morai_msgs::SkidSteer6wUGVCtrlCmd escape_ctrl;
    escape_ctrl.cmd_type = 3;
    escape_ctrl.Target_linear_velocity = 0;
    escape_ctrl.Target_angular_velocity = -0.83;  // 최대 각속도로 왼쪽 회전 (0.83 rad/s)
    
    double rotation_time = M_PI/2 / 0.83;  // 90도 (1.57 rad)를 0.83 rad/s로 회전하는 데 필요한 시간
    ros::Time start_time = ros::Time::now();

    ros::Rate rate(10);  // 10 Hz
    while (ros::ok() && (ros::Time::now() - start_time).toSec() < rotation_time) {
        escape_ctrl_pub_.publish(escape_ctrl);  // 회전 명령 퍼블리시
        rate.sleep();
    }

    Brake();  // 회전 후 정지

    ROS_INFO("Turned Left 90 Degrees!");
}

void DynamicPlanning::TurnLeft180() {
    morai_msgs::SkidSteer6wUGVCtrlCmd escape_ctrl;
    escape_ctrl.cmd_type = 3;
    escape_ctrl.Target_linear_velocity = 0;
    escape_ctrl.Target_angular_velocity = -0.83;  // 최대 각속도로 회전 (0.83 rad/s)

    double rotation_time = M_PI / 0.83 + 0.146;  // 180도 (3.14 rad)를 0.83 rad/s로 회전하는 데 필요한 시간
    ros::Time start_time = ros::Time::now();

    ros::Rate rate(10);  
    while (ros::ok() && (ros::Time::now() - start_time).toSec() < rotation_time) {
        escape_ctrl_pub_.publish(escape_ctrl);  // 회전 명령 퍼블리시
        rate.sleep();
    }

    Brake();  // 회전 후 정지
    ROS_INFO("Turned 180 Degrees!");
}
void DynamicPlanning::TurnRight180() {
    morai_msgs::SkidSteer6wUGVCtrlCmd escape_ctrl;
    escape_ctrl.cmd_type = 3;
    escape_ctrl.Target_linear_velocity = 0;
    escape_ctrl.Target_angular_velocity = 0.83;  // 최대 각속도로 회전 (0.83 rad/s)

    double rotation_time = M_PI / 0.83 - 0.0156;  // 180도 (3.14 rad)를 0.83 rad/s로 회전하는 데 필요한 시간
    ros::Time start_time = ros::Time::now();

    ros::Rate rate(10);  
    while (ros::ok() && (ros::Time::now() - start_time).toSec() < rotation_time) {
        escape_ctrl_pub_.publish(escape_ctrl);  // 회전 명령 퍼블리시
        rate.sleep();
    }

    Brake();  // 회전 후 정지
    ROS_INFO("Turned Right 180 Degrees!");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "escape_ctrl_node");
    ros::NodeHandle nh;

    DynamicPlanning dp;
        
    dp.Rear();
    // ros::Duration(0.5).sleep();  
    // dp.TurnLeft180();
    // ros::Duration(0.5).sleep();  
    // //dp.TurnRight180();
    // dp.Brake();

    ros::Rate loop_rate(10); 
    ros::spin();
    return 0;
}