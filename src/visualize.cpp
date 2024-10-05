#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <Eigen/Dense>
#include <tf/transform_datatypes.h> // tf를 사용하여 쿼터니언 변환

struct Waypoint{
    int idx;
    float x;
    float y;
    float heading;
};

class Visualize {
public:
    Visualize(ros::NodeHandle& nh) {
        pose_subscriber = nh.subscribe("/current_pose", 10, &Visualize::poseCallback, this);
        lidar_subscriber = nh.subscribe("/lidar_utm", 10, &Visualize::lidarCallback, this);
        waypoint_marker_pub = nh.advertise<visualization_msgs::Marker>("waypoints", 10);
        transformed_lidar_pub = nh.advertise<sensor_msgs::PointCloud2>("transformed_lidar", 10);
        current_pose_marker_pub = nh.advertise<visualization_msgs::Marker>("current_pose_marker", 10);
        
        // 파라미터로 CSV 파일 경로를 받음
        nh.param<std::string>("/visualize/waypoint_file", waypoint_file, "waypoints.csv");
        
        // 웨이포인트 로드
        loadWaypoints();
    }

private:
    ros::Subscriber pose_subscriber;
    ros::Subscriber lidar_subscriber;
    ros::Publisher waypoint_marker_pub;
    ros::Publisher transformed_lidar_pub;
    ros::Publisher current_pose_marker_pub;

    geometry_msgs::Pose2D current_pose;
    std::string waypoint_file;

    std::vector<Waypoint> waypoints;

    void poseCallback(const geometry_msgs::Pose2D::ConstPtr& msg) {
        current_pose = *msg;        
        // 웨이포인트를 변환하여 RViz에 표시
        publishWaypoints();
        publishCurrentPose(); // 현재 포즈를 RViz에 표시
    }

    void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud); // ROS 메시지를 PCL 형식으로 변환
        
        pcl::PointCloud<pcl::PointXYZ> transformed_cloud;

        // 현재 포즈를 기준으로 포인트 변환
        Eigen::Matrix2d rotation;
        rotation << cos(-current_pose.theta), -sin(-current_pose.theta),
                    sin(-current_pose.theta), cos(-current_pose.theta);

        for (const auto& point : cloud.points) {
            Eigen::Vector2d original_point(point.x, point.y);
            Eigen::Vector2d transformed_point = rotation * (original_point - Eigen::Vector2d(current_pose.x, current_pose.y));
            //ROS_INFO("lidar Pose: x = %.2f, y = %.2f, yaw = %.2f", transformed_point(0), transformed_point(1), 0);

            pcl::PointXYZ new_point;
            new_point.x = transformed_point(0);
            new_point.y = transformed_point(1);
            new_point.z = point.z; // Z축은 그대로 유지
            transformed_cloud.points.push_back(new_point);
        }

        // transformed_cloud.width = transformed_cloud.points.size();
        // transformed_cloud.height = 1; // 단일 포인트 클라우드
        // transformed_cloud.is_dense = true;

        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(transformed_cloud, output);
        output.header.frame_id = "map"; // RViz에서 사용할 프레임
        output.header.stamp = ros::Time::now();
        transformed_lidar_pub.publish(output); // 변환된 포인트 클라우드 퍼블리시
    }

    void loadWaypoints() {
    
        int cnt = 0;
        
        std::ifstream file(waypoint_file);
        if (file.is_open()) {

            ROS_INFO("ROAD : %s",waypoint_file);

            std::string line;

            // CSV 파일의 각 줄을 읽음
            while (std::getline(file, line)) {

                if (cnt == 0) {
                    // 첫 줄은 데이터 프레임
                    cnt ++;
                    continue;
                }
                Waypoint wp;

                std::istringstream ss(line);
                std::string token;

                std::getline(ss, token, ','); // x
                wp.x = std::stod(token);
                std::getline(ss, token, ','); // y
                wp.y = std::stod(token);
                std::getline(ss, token, ','); // heading
                wp.heading = std::stod(token);
                std::getline(ss, token, ','); // index
                wp.idx = std::stoi(token);
                
                waypoints.push_back(wp);

                cnt ++;
            }
        }
    }

    void publishWaypoints() {

        int cnt = 0;

        visualization_msgs::Marker marker;
        marker.header.frame_id = "map"; // RViz에서 사용할 프레임
        marker.header.stamp = ros::Time::now();
        marker.ns = "waypoints";
        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::SPHERE_LIST;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        Eigen::Matrix2d rotation;
        rotation << cos(-current_pose.theta), -sin(-current_pose.theta),
                    sin(-current_pose.theta), cos(-current_pose.theta);

        for (const auto& waypoint : waypoints) {
            geometry_msgs::Point p;

            Eigen::Vector2d original_point(waypoint.x,  waypoint.y);
            Eigen::Vector2d transformed_point = rotation * (original_point - Eigen::Vector2d(current_pose.x, current_pose.y));

            p.x = transformed_point(0);
            p.y = transformed_point(1);
            p.z = 0; // Z축은 0으로 설정

            marker.points.push_back(p);
        }

        waypoint_marker_pub.publish(marker);
    }

    void publishCurrentPose() {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map"; // RViz에서 사용할 프레임
        marker.header.stamp = ros::Time::now();
        marker.ns = "current_pose";
        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::ARROW; // 화살표로 현재 포즈 표시
        marker.scale.x = 0.7;
        marker.scale.y = 0.3; 
        marker.scale.z = 0.3; 
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        // 현재 포즈의 위치
        marker.pose.position.x = 0;//current_pose.x;
        marker.pose.position.y = 0;//current_pose.y;
        marker.pose.position.z = 0; // Z축은 0으로 설정

        // Yaw 각도에 따라 방향 설정
        tf::Quaternion q = tf::createQuaternionFromYaw(0);
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();

        current_pose_marker_pub.publish(marker); // 현재 포즈 마커 퍼블리시
    }

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "node_name");
    ros::NodeHandle nh;

    Visualize visualize(nh);

    ros::spin(); // 콜백 대기
    return 0;
}
