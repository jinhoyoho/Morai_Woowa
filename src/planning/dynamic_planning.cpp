#include "Morai_Woowa/dynamic_planning.h"

DynamicPlanning::DynamicPlanning() : nh_("~") {
    ctrl_cmd_pub_ = nh_.advertise<morai_msgs::SkidSteer6wUGVCtrlCmd>("/6wheel_skid_ctrl_cmd", 10);
}

void DynamicPlanning::Brake() {
    morai_msgs::SkidSteer6wUGVCtrlCmd ctrl_cmd;
    ctrl_cmd.cmd_type = 3;
    ctrl_cmd.Target_linear_velocity = 0;
    ctrl_cmd.Target_angular_velocity = 0;
    ctrl_cmd_pub_.publish(ctrl_cmd);
    ROS_INFO("Brake Activated!");
}

void DynamicPlanning::Rear() {
    morai_msgs::SkidSteer6wUGVCtrlCmd ctrl_cmd;
    ctrl_cmd.cmd_type = 3;
    ctrl_cmd.Target_linear_velocity = -1;  // 후진 속도 -1 m/s
    ctrl_cmd.Target_angular_velocity = 0;

    double rear_time = 1.0;  // 1m를 이동하는데 필요한 시간 (속도 -1 m/s로 1초)
    ros::Time start_time = ros::Time::now();

    ros::Rate rate(10);  // 10 Hz
    while (ros::ok() && (ros::Time::now() - start_time).toSec() < rear_time) {
        ctrl_cmd_pub_.publish(ctrl_cmd);  // 후진 명령 퍼블리시
        rate.sleep();
    }

    Brake();  // 후진 후 정지
    ROS_INFO("Moved 1m Backwards and Stopped!");
}

void DynamicPlanning::TurnRight90() {
    morai_msgs::SkidSteer6wUGVCtrlCmd ctrl_cmd;
    ctrl_cmd.cmd_type = 3;
    ctrl_cmd.Target_linear_velocity = 0;
    ctrl_cmd.Target_angular_velocity = -0.83;  // 최대 각속도로 오른쪽 회전 (0.83 rad/s)
    
    double rotation_time = 1.57 / 0.83;  // 90도 (1.57 rad)를 0.83 rad/s로 회전하는 데 필요한 시간
    ros::Time start_time = ros::Time::now();

    ros::Rate rate(10);  // 10 Hz
    while (ros::ok() && (ros::Time::now() - start_time).toSec() < rotation_time) {
        ctrl_cmd_pub_.publish(ctrl_cmd);  // 회전 명령 퍼블리시
        rate.sleep();
    }

    Brake();  // 회전 후 정지
    ROS_INFO("Turned Right 90 Degrees!");
}

void DynamicPlanning::TurnLeft90() {
    morai_msgs::SkidSteer6wUGVCtrlCmd ctrl_cmd;
    ctrl_cmd.cmd_type = 3;
    ctrl_cmd.Target_linear_velocity = 0;
    ctrl_cmd.Target_angular_velocity = 0.83;  // 최대 각속도로 왼쪽 회전 (0.83 rad/s)
    
    double rotation_time = 1.57 / 0.83;  // 90도 (1.57 rad)를 0.83 rad/s로 회전하는 데 필요한 시간
    ros::Time start_time = ros::Time::now();

    ros::Rate rate(10);  // 10 Hz
    while (ros::ok() && (ros::Time::now() - start_time).toSec() < rotation_time) {
        ctrl_cmd_pub_.publish(ctrl_cmd);  // 회전 명령 퍼블리시
        rate.sleep();
    }

    Brake();  // 회전 후 정지
    ROS_INFO("Turned Left 90 Degrees!");
}

void DynamicPlanning::Turn180() {
    morai_msgs::SkidSteer6wUGVCtrlCmd ctrl_cmd;
    ctrl_cmd.cmd_type = 3;
    ctrl_cmd.Target_linear_velocity = 0;
    ctrl_cmd.Target_angular_velocity = 0.83;  // 최대 각속도로 회전 (0.83 rad/s)

    double rotation_time = 3.14 / 0.83;  // 180도 (3.14 rad)를 0.83 rad/s로 회전하는 데 필요한 시간
    ros::Time start_time = ros::Time::now();

    ros::Rate rate(10);  // 10 Hz
    while (ros::ok() && (ros::Time::now() - start_time).toSec() < rotation_time) {
        ctrl_cmd_pub_.publish(ctrl_cmd);  // 회전 명령 퍼블리시
        rate.sleep();
    }

    Brake();  // 회전 후 정지
    ROS_INFO("Turned 180 Degrees!");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "dynamic_planning_node");
    ros::NodeHandle nh;

    DynamicPlanning dp;
    
    ros::Rate loop_rate(10);  // 10 Hz
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}