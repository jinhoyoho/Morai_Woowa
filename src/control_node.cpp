#include "Morai_Woowa/control_node.h"

control_node::control_node(ros::NodeHandle& nh)
{
    ROS_INFO("Mode: 1");
    prev_mode = 0;
    mode = 1; // path_tracking mode로 초기화
    cmd_type = 3;
    Target_angular_velocity = 0; // 각속도 0으로 초기화
    Target_linear_velocity = 0; // 선속도 0으로 초기화

    path_tracking_sub_ = nh.subscribe("/path_tracking_ctrl", 1, &control_node::callBack_1, this);
    collision_sub_ = nh.subscribe("/collision_ctrl", 1, &control_node::callBack_2, this);
    escape_sub_ = nh.subscribe("/escape_ctrl", 1, &control_node::callBack_3, this);
    state_sub_ = nh.subscribe("/state_ctrl", 1, &control_node::callBack_4, this);
    control_pub_ = nh.advertise<morai_msgs::SkidSteer6wUGVCtrlCmd>("/6wheel_skid_ctrl_cmd", 10);
    mode_pub_ = nh.advertise<std_msgs::Int8>("/mode", 10);

    
    // 서비스 서버
    control_server_ = nh.advertiseService("/Control_srv", &control_node::change_mode, this);
}

void control_node::callBack_1(const morai_msgs::SkidSteer6wUGVCtrlCmdPtr& msg)
{
    // plannig_tracking mode일 때 실행
    if(mode == 1)
    {
        ROS_INFO("Mode: %d", mode);
        
        cmd_type = msg->cmd_type;
        Target_angular_velocity = msg->Target_angular_velocity;
        Target_linear_velocity = msg->Target_linear_velocity;
    }
}

void control_node::callBack_2(const morai_msgs::SkidSteer6wUGVCtrlCmdPtr& msg)
{
    // collision일 때 실행
    if(mode == 2)
    {
        ROS_INFO("Mode: %d", mode);
        
        cmd_type = msg->cmd_type;
        Target_angular_velocity = msg->Target_angular_velocity;
        Target_linear_velocity = msg->Target_linear_velocity;
    }
}

void control_node::callBack_3(const morai_msgs::SkidSteer6wUGVCtrlCmdPtr& msg)
{
    // escape일 때 실행
    if(mode == 3)
    {
        ROS_INFO("Mode: %d", mode);

        cmd_type = msg->cmd_type;
        Target_angular_velocity = msg->Target_angular_velocity;
        Target_linear_velocity = msg->Target_linear_velocity;
    }
}

void control_node::callBack_4(const morai_msgs::SkidSteer6wUGVCtrlCmdPtr& msg)
{
    // plannig_tracking mode일 때 실행
    if(mode == 4)
    {
        ROS_INFO("Mode: %d", mode);

        cmd_type = msg->cmd_type;
        Target_angular_velocity = msg->Target_angular_velocity;
        Target_linear_velocity = msg->Target_linear_velocity;
    }
}

void control_node::publish_ctrl()
{
    std_msgs::Int8 mode_msg;
    mode_msg.data = mode;
    mode_pub_.publish(mode_msg);

    morai_msgs::SkidSteer6wUGVCtrlCmd msg;
    msg.cmd_type = cmd_type;
    msg.Target_angular_velocity = Target_angular_velocity;
    msg.Target_linear_velocity = Target_linear_velocity;
    control_pub_.publish(msg);
}

bool control_node::change_mode(morai_woowa::ControlSrv::Request &req, morai_woowa::ControlSrv::Response &res)
{
    ROS_INFO("Change mode!");

    if(req.mode == 0)   // 0번이면
    {
        mode = prev_mode;   // 이전 모드 저장
    }
    else
    {
        prev_mode = mode;   // 현재 모드를 이전 노드로 저장
        mode = req.mode;    // 현재 모드 갱신
    }

    ROS_INFO("Mode: %d", mode);

    res.result = true;

    return res.result;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "control_node");
    ros::NodeHandle nh;

    ROS_INFO("Control_node ON!");
    control_node cn(nh);
    
    ros::Rate rate(20);  // 0.01 Hz

    while(ros::ok())
    {
        cn.publish_ctrl();   // publish하기

        ros::spinOnce();
        
        rate.sleep();  // 지정된 주기로 대기

    }

    return 0;
}