#include <ros/ros.h>

#include <morai_msgs/SkidSteer6wUGVCtrlCmd.h>
#include "morai_woowa/ControlSrv.h"  // mode: 0, 1, 2, 3을 받음, result: 잘 받았는지 확인

class control_node
{
    private:
        ros::Subscriber path_tracking_sub_; // 6wheel_skid_ctrl_cmd 토픽 받기
        ros::Subscriber collision_sub_; // 6wheel_skid_ctrl_cmd 토픽 받기
        ros::Subscriber escape_sub_; // 6wheel_skid_ctrl_cmd 토픽 받기
        ros::Publisher control_pub_; // simulation으로 publish
        ros::ServiceServer control_server_; // control server

        int mode; // 현재 모드
        int prev_mode;

        // simulation에 publish할 변수들
        int cmd_type;
        float Target_linear_velocity;
        float Target_angular_velocity;

    public:
        control_node(ros::NodeHandle& nh);  // 생성자

        void callBack_1(const morai_msgs::SkidSteer6wUGVCtrlCmdPtr& msg);
        void callBack_2(const morai_msgs::SkidSteer6wUGVCtrlCmdPtr& msg);
        void callBack_3(const morai_msgs::SkidSteer6wUGVCtrlCmdPtr& msg);
        bool change_mode(morai_woowa::ControlSrv::Request &req, morai_woowa::ControlSrv::Response &res);
        void publish_ctrl();
};