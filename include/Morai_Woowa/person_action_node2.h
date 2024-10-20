#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "morai_woowa/Person_Collision_Act2Action.h"
#include <morai_msgs/SkidSteer6wUGVCtrlCmd.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>  // tf 관련 함수 사용을 위한 헤더 추가
#include <nav_msgs/Path.h>  // nav_msgs::Path 메시지 사용을 위한 헤더
#include <cmath>

float LIN_VEL(7.2);

struct Spot {
    float x;
    float y;
    Spot() : x(0.0), y(0.0) {}
    Spot(float x_coord, float y_coord) : x(x_coord), y(y_coord) {}
};


class person_action_node
{
private:
    ros::Publisher ctrl_cmd_pub_; 
    ros::Publisher path_pub_; 

    ros::Subscriber current_pose_sub_;

    actionlib::SimpleActionServer<morai_woowa::Person_Collision_Act2Action> PCAserver_;

    float current_x_;
    float current_y_;
    float current_yaw_;

public:
    person_action_node(ros::NodeHandle& nh);

    void execute(const morai_woowa::Person_Collision_Act2GoalConstPtr& goal);
    void currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg); 
    std::vector<Spot> generateSemiCircularPath(const Spot& start, const Spot& goal, int num_points);
    Spot load_spot(int spot, bool is_indoor);
    //void currentPoseCallback(const morai_woowa::average_points_array::ConstPtr& msg); 
    bool check_collision_success();
    void publishPath(const std::vector<Spot>& path);
    double steering_angle();
    void followPath();
    double calculateCurvature();


    int cnt;
    Spot apex;  // 삼각형 꼭짓점(경유 지점)



    std::vector<Spot> path_;

    // 두 점 사이의 거리 계산 함수
    double calculateDistance(const Spot& p1, const Spot& p2) {
        return std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2));
    }
    double calculateDistance(double x1, double y1, double x2, double y2) {
        return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
    }
};