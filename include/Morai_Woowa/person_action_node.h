#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "morai_woowa/Person_Collision_Act2Action.h"
#include <morai_msgs/SkidSteer6wUGVCtrlCmd.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h> // 쿼터니언 변환을 위해 사용
#include "Morai_Woowa/AStar.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/conversions.h>

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cmath>


int ANG_VEL(0.4);
int LIN_VEL(7.2);


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
    ros::Publisher ctrl_cmd_pub_; 
    ros::Publisher astar_path_pub_; 
    ros::Publisher marker_pub_;
    ros::Subscriber current_pose_sub_;
    ros::Subscriber lidar_utm_sub;    // 라이다 좌표
    
    actionlib::SimpleActionServer<morai_woowa::Person_Collision_Act2Action> PCAserver_;
    AStar::Generator generator;

    person closest_person_;
    float person_range_;
    bool is_target;
    double averageDistance;
    ros::Time last_target_found_time_;

    float current_x_;
    float current_y_;
    double current_yaw_;

    float grid_size_;

    float world_size_limit_;

    AStar::Vec2i world_x_limit_;
    AStar::Vec2i world_y_limit_;
    AStar::Vec2i goal_person_;
    std::vector<std::pair<float, float>> astar_point_vector_;

public:
    person_action_node(ros::NodeHandle& nh);

    void execute(const morai_woowa::Person_Collision_Act2GoalConstPtr& goal);
    void currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg); 
    bool check_collision_success();
    void pub_marker(float LD_x, float LD_y);
    void pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    AStar::Vec2i load_spot(int spot, bool is_indoor);

    double calculateDistance(double x1, double y1, double x2, double y2) {
        return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
    }
};