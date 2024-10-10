#include <ros/ros.h>
#include <ros/package.h>  
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include "morai_msgs/SkidSteer6wUGVCtrlCmd.h"
#include "morai_msgs/WoowaDillyEventCmdSrv.h"
#include "morai_msgs/DillyCmd.h"
#include "morai_msgs/DillyCmdResponse.h"

#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h> // tf를 사용하여 쿼터니언 변환

#include <vector>
#include <fstream>
#include <sstream>
#include <cmath>
#include <limits>
#include <string>
#include <filesystem>
#include <thread>  // 스레드 사용을 위한 헤더

struct Waypoint {
    double x;
    double y;
    double heading;
    int index;
};

typedef std::vector<Waypoint> path;

class PurePursuit {
public:
    PurePursuit() : wheel_base(0.6), lfd(2.0), steering(0), is_look_forward_point(false) {}

    // 경로를 UTMK 좌표계로 설정
    void getPath(const nav_msgs::Path& msg) {
        waypoint_path.clear(); // 기존 경로 초기화
        for (const auto& pose : msg.poses) {
            Waypoint wp;
            wp.x = pose.pose.position.x;
            wp.y = pose.pose.position.y;
            wp.heading = tf::getYaw(pose.pose.orientation);  // 경로의 heading을 yaw로 변환
            waypoint_path.push_back(wp);
        }
    }
    void getEgoStatus(const geometry_msgs::PoseStamped& ego_status, double current_vel) {
        current_position.x = ego_status.pose.position.x;
        current_position.y = ego_status.pose.position.y;
        
        // Quaternion to yaw
        tf::Quaternion q(
            ego_status.pose.orientation.x,
            ego_status.pose.orientation.y,
            ego_status.pose.orientation.z,
            ego_status.pose.orientation.w
        );
        tf::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, vehicle_yaw);  // vehicle_yaw in radians
        this->current_vel = current_vel;  // Velocity in kph
    
        // lfd를 동적으로 조정 (기본 lfd 2.0 + 속도 계수 0.5 적용)
        lfd = 2.0 + 0.5 * current_vel;
        ROS_INFO("Dynamic LFD: %.2f", lfd);
    
    }

    double steering_angle(double lfd_input) {
        lfd = lfd_input;  // Look-forward distance를 업데이트
        is_look_forward_point = false;
        geometry_msgs::Point rotated_point;

        for (const auto& wp : waypoint_path) {
            // 좌표 변환: 로봇 기준으로 경로점 위치 변환
            double dx = wp.x - current_position.x;
            double dy = wp.y - current_position.y;

            rotated_point.x = cos(vehicle_yaw) * dx + sin(vehicle_yaw) * dy;
            rotated_point.y = sin(vehicle_yaw) * dx - cos(vehicle_yaw) * dy;

            if (rotated_point.x > 0) {
                double dis = sqrt(pow(rotated_point.x, 2) + pow(rotated_point.y, 2));
                if (dis >= lfd) {
                    forward_point = wp;
                    is_look_forward_point = true;
                    break;
                }
            }
        }

        // 스티어링 각도 계산
        if (is_look_forward_point) {
            double theta = atan2(rotated_point.y, rotated_point.x);
            steering = atan2(2 * wheel_base * sin(theta), lfd) * 180.0 / M_PI;
        } else {
            steering = 0;
        }

        return steering;
    }

private:
    Waypoint forward_point;
    geometry_msgs::Point current_position;
    double vehicle_yaw;
    double current_vel;
    double wheel_base;
    double lfd; // 동적으로 변경될 look-ahead distance
    double steering;
    bool is_look_forward_point;
    path waypoint_path;
};

class StateNode {
public:
    StateNode() : closest_index_(-1) {

        //path폴더안에 path파일 이름!!!!!
        std::string filename = "test_path.csv";
        std::string current_path = ros::package::getPath("morai_woowa"); // 패키지 경로를 가져옵니다
        waypoint_file_ = current_path + "/path/" + filename;

        nh_.param<std::string>("/state_node/waypoint_file1", waypoint_file_1_, "waypoints.csv");
        nh_.param<std::string>("/state_node/waypoint_file2", waypoint_file_2_, "waypoints.csv");
        nh_.param<std::string>("/state_node/waypoint_file3", waypoint_file_3_, "waypoints.csv");

        delivery_pickup_client_ = nh_.serviceClient<morai_msgs::WoowaDillyEventCmdSrv>("/WoowaDillyEventCmd");

        current_pose_sub_ = nh_.subscribe("/current_pose", 10, &StateNode::currentPoseCallback, this);
        path_pub_ = nh_.advertise<nav_msgs::Path>("/global_path", 10); // 경로 퍼블리셔
        waypoint_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/globalpath", 10, this);
        ctrl_cmd_pub= nh_.advertise<morai_msgs::SkidSteer6wUGVCtrlCmd>("/6wheel_skid_ctrl_cmd", 10); // Skid-Steer Control Publisher


        loadWaypoints(waypoint_file_1_, waypoints_1_);  // waypoint 파일에서 로드
        loadWaypoints(waypoint_file_2_, waypoints_2_);  // waypoint 파일에서 로드
        loadWaypoints(waypoint_file_3_, waypoints_3_);  // waypoint 파일에서 로드
    }
    PurePursuit pure_pursuit;
    ros::Publisher ctrl_cmd_pub;

    void currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {

        closest_index_ = findClosestWaypoint(msg->pose.position.x, msg->pose.position.y, closest_index_, waypoints_1_);
        
        if (closest_index_ != -1) {
            ROS_INFO("Current Position: (X: %.2f, Y: %.2f) is nearest to Waypoint Index: %d", msg->pose.position.x, msg->pose.position.y, closest_index_);
            ROS_INFO("cross_track_error: %lf", cross_track_error);
        } else {
            ROS_WARN("No waypoints available.");
        }
    }

    void pub_global_path(){
        publishPath(waypoints_1_);
        publishPath(waypoints_2_);
        publishPath(waypoints_3_);

        publishWaypoints(waypoints_1_);
        publishWaypoints(waypoints_2_);
        publishWaypoints(waypoints_3_);
    }
    void publishPath(const path& waypoints) {
    // nav_msgs::Path 메시지 생성
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "map"; // 사용할 프레임
    path_msg.header.stamp = ros::Time::now();

    for (const auto& waypoint : waypoints) {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = "map";
        pose_stamped.header.stamp = ros::Time::now();
        
        pose_stamped.pose.position.x = waypoint.x;
        pose_stamped.pose.position.y = waypoint.y;
        pose_stamped.pose.position.z = 0;

        // heading 정보를 쿼터니언으로 변환
        tf::Quaternion q = tf::createQuaternionFromYaw(waypoint.heading);
        pose_stamped.pose.orientation.x = q.x();
        pose_stamped.pose.orientation.y = q.y();
        pose_stamped.pose.orientation.z = q.z();
        pose_stamped.pose.orientation.w = q.w();

        path_msg.poses.push_back(pose_stamped);
    }

    // 퍼블리시 (경로 추적에 사용)
    path_pub_.publish(path_msg);
    }
    void publishWaypoints(path waypoints) {

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

        for (const auto& waypoint : waypoints) {
            geometry_msgs::Point p;
            p.x = waypoint.x;
            p.y = waypoint.y;
            p.z = 0; // Z축은 0으로 설정

            marker.points.push_back(p);
        }

        waypoint_marker_pub_.publish(marker);
    }

    bool delivery(int item_index){
        
        morai_msgs::WoowaDillyEventCmdSrv deli_srv;
        deli_srv.request.request.isPickup = true; 
        deli_srv.request.request.deliveryItemIndex = item_index; 
        
        delivery_pickup_client_.call(deli_srv);
        
        ros::Rate loop_rate(0.5);

        int cnt = 0;

        while (!deli_srv.response.response.result && ros::ok() && cnt < 5){
            std::cout << "state node : Failed to delivery. Retrying..." << std::endl;
            delivery_pickup_client_.call(deli_srv);
            loop_rate.sleep();
            ros::spinOnce();
            cnt ++;
        }

        return deli_srv.response.response.result;
        
        // morai_msgs::DillyCmd srv;
        // srv.isPickup = false;  // 또는 false
        // srv.deliveryItemIndex = item_index;  // 원하는 인덱스

        // // 서비스 호출
        // if (delivery_pickup_client_.call(srv)) {
        //     ROS_INFO("Result: %s", srv.response.result ? "true" : "false");
        // } else {
        //     ROS_ERROR("Failed to call service dilly_cmd_service");
        // }
    } 

    bool pickup(int item_index){

        morai_msgs::WoowaDillyEventCmdSrv pick_srv;
        pick_srv.request.request.deliveryItemIndex = item_index;
        pick_srv.request.request.isPickup = true;

        delivery_pickup_client_.call(pick_srv);

        ros::Rate loop_rate(0.5);

        int cnt = 0;

        while (!pick_srv.response.response.result && ros::ok() && cnt < 5){
            std::cout << "state node : Failed to pick up. Retrying..." << std::endl;
            delivery_pickup_client_.call(pick_srv);
            loop_rate.sleep();
            ros::spinOnce();
            cnt ++;
        }

        return pick_srv.response.response.result;
        
        // morai_msgs::DillyCmd srv;
        // srv.isPickup = true;  // 또는 false
        // srv.deliveryItemIndex = item_index;  // 원하는 인덱스

        // // 서비스 호출
        // if (delivery_pickup_client_.call(srv)) {
        //     ROS_INFO("Result: %s", srv.response.result ? "true" : "false");
        // } else {
        //     ROS_ERROR("Failed to call service dilly_cmd_service");
        // }

    }


    // 여기가 메인 state 함수임!!
    void state() {
        ros::Rate rate(100);  // 0.01 Hz
        while (ros::ok()) {

            ////// 여기다 실행할 함수 //////
        if (closest_index_ != -1) {
                // Pure Pursuit Logic and Control Commands
                double lfd = 2.0;
                double steering_angle = pure_pursuit.steering_angle(lfd);
                double target_linear_velocity = 2.0;  
                double target_angular_velocity = steering_angle * M_PI / 180.0;

                morai_msgs::SkidSteer6wUGVCtrlCmd ctrl_cmd_msg;
                ctrl_cmd_msg.cmd_type = 3;
                ctrl_cmd_msg.Target_linear_velocity = target_linear_velocity;  
                ctrl_cmd_msg.Target_angular_velocity = target_angular_velocity;

                ctrl_cmd_pub.publish(ctrl_cmd_msg);  // Publish Control Command

                ROS_INFO("Publishing Control Command: Linear Velocity: %.2f, Angular Velocity: %.2f", target_linear_velocity, target_angular_velocity);
            }
            ROS_INFO("closest_index: %d", closest_index_);


            pub_global_path();
            rate.sleep();  // 지정된 주기로 대기
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber current_pose_sub_;
    ros::Publisher path_pub_; // nav_msgs::Path 퍼블리셔
    ros::Publisher waypoint_marker_pub_;
    ros::ServiceClient delivery_pickup_client_;

    std::string waypoint_file_;  // CSV 파일 경로를 저장할 변수
    path waypoints_1_;
    path waypoints_2_;
    path waypoints_3_;

    std::string waypoint_file_1_;
    std::string waypoint_file_2_;
    std::string waypoint_file_3_;

    int closest_index_ = 0;
    double cross_track_error = 10000000; 

    bool wpt_init_flag = false;

    void loadWaypoints(std::string waypoint_file_, path &waypoints_) {
        
        int cnt = 0;

        std::ifstream file(waypoint_file_);
        if (file.is_open()) {
            Waypoint wp;
            std::string line;

            // CSV 파일의 각 줄을 읽음
            while (std::getline(file, line)) {

                if (cnt == 0) {
                    // 첫 줄은 데이터 프레임
                    cnt ++;
                    continue;
                }

                std::istringstream ss(line);
                std::string token;

                std::getline(ss, token, ','); // x
                wp.x = std::stod(token);
                std::getline(ss, token, ','); // y
                wp.y = std::stod(token);
                std::getline(ss, token, ','); // heading
                wp.heading = std::stod(token);
                std::getline(ss, token, ','); // index
                wp.index = std::stoi(token);
                
                waypoints_.push_back(wp);

                cnt ++;
            }
            file.close();
            ROS_INFO("Loaded waypoints from %s", waypoint_file_.c_str());
        } else {
            ROS_WARN("Unable to open waypoint file: %s", waypoint_file_.c_str());
        }
    }

    int findClosestWaypoint(double x, double y, int previous_index, path waypoints) {

        // 처음엔 모든 wpt를 돌면서 위치 체크
        if(!wpt_init_flag){
            int closest_index = -1;
            double min_distance = std::numeric_limits<double>::max();

            for (int i = 0; i < waypoints.size(); i++) {
                const auto& wp = waypoints[i];
                double distance = calculateDistance(x, y, wp.x, wp.y);
                if (distance < min_distance) {
                    min_distance = distance;
                    closest_index = wp.index;
                    cross_track_error = min_distance;
                }
            }

            wpt_init_flag = true;

            return closest_index;
        }

        // 이후부터는 연산량을 위해 이전 wpt 앞뒤로 5개 점만 보고 가까운 idx찾기
        int start_index = std::max(previous_index - 5, 0);
        int end_index = std::min(previous_index + 5, static_cast<int>(waypoints.size()) - 1);

        int closest_index = -1;
        double min_distance = std::numeric_limits<double>::max();

        for (int i = start_index; i <= end_index; ++i) {

            const auto& wp = waypoints[i];
            double distance = calculateDistance(x, y, wp.x, wp.y);
            if (distance < min_distance) {
                min_distance = distance;
                closest_index = wp.index;
                cross_track_error = min_distance;
            }
        }

        return closest_index;
    }

    double calculateDistance(double x1, double y1, double x2, double y2) {
        return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "state_node");

    StateNode StateNode;

    std::thread thread(&StateNode::state, &StateNode);

    ros::spin();
    
    thread.join();  // 스레드가 종료될 때까지 대기

    return 0;
}