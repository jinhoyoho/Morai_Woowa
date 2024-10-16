#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf/tf.h>
#include <actionlib/server/simple_action_server.h>
#include <morai_woowa/Planning_Tracking_ActAction.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <algorithm>  // std::reverse

//2024 10 01 dwa가 동적 장애물을 받고 적용하도록 수정
//2024 10 12 dwa global frame

int num_of_path = 15;
double predict_time = 3;
double velocity = 1.0;
double angular_velocity = 0.5;
double obstacle_cost = 2.0;
double global_path_cost = 0.1;
double progress = 0.0;

const double PI = 3.1415926;

double calculateDistance(const std::vector<double>& point1, const std::vector<double>& point2);
void obstacle_callback(const sensor_msgs::PointCloud2::ConstPtr& msg);
void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
void gpath_callback(const nav_msgs::Path::ConstPtr& msg);
void filter_obstacle();
void load_gpath(std::string path_name, bool is_reverse);
void gpath_cut();
void make_candidate_path(std::shared_ptr<std::vector<std::vector<std::vector<double>>>> candidate_path_ptr);
void vote_president_path(std::shared_ptr<std::vector<std::vector<std::vector<double>>>> candidate_path_ptr);
void make_msg(std::shared_ptr<std::vector<std::vector<std::vector<double>>>> candidate_path_ptr, 
             std::shared_ptr<std::vector<std::vector<double>>> president_path_ptr,
             std::shared_ptr<sensor_msgs::PointCloud> candidate_msg_ptr, std::shared_ptr<nav_msgs::Path> president_msg_ptr);

void executeCB(const morai_woowa::Planning_Tracking_ActGoalConstPtr &goal);

auto obstacle_ptr = std::make_shared<std::vector<std::vector<double>>>();
auto pose_ptr = std::make_shared<std::vector<double>>(3);
auto global_path_ptr = std::make_shared<std::vector<std::vector<double>>>();
auto president_path_ptr = std::make_shared<std::vector<std::vector<double>>>();
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
int best_idx = int(num_of_path*0.5) + 1;
bool goal_received = false;

morai_woowa::Planning_Tracking_ActActionResult pt_result;
morai_woowa::Planning_Tracking_ActActionFeedback pt_feedback;
actionlib::SimpleActionServer<morai_woowa::Planning_Tracking_ActAction> planning_tracking_as;

int main(int argc, char **argv){
    ros::init(argc, argv, "dwa");
    ros::NodeHandle nh;
    actionlib::SimpleActionServer<morai_woowa::Planning_Tracking_ActAction> planning_tracking_as(nh, "planning_tracking", executeCB, false);
    ros::Rate loop_rate(10);
    ros::Publisher candidate_pub = nh.advertise<sensor_msgs::PointCloud>("candidate_path", 100);
    ros::Publisher president_pub = nh.advertise<nav_msgs::Path>("lpath", 100);
    ros::Subscriber obstacle_sub = nh.subscribe("obstacle", 100, obstacle_callback);
    ros::Subscriber pose_sub = nh.subscribe("current_pose", 100, pose_callback);
    // ros::Subscriber global_path = nh.subscribe("/gpath", 100, gpath_callback);

    auto candidate_path_ptr = std::make_shared<std::vector<std::vector<std::vector<double>>>>();

    planning_tracking_as.start();
    while(ros::ok()){
        // std::cout<<global_path_ptr->size()<<std::endl;
        if(global_path_ptr->size()>0){
            gpath_cut();
            make_candidate_path(candidate_path_ptr);

            filter_obstacle();
            
            vote_president_path(candidate_path_ptr);
            auto candidate_msg_ptr = std::make_shared<sensor_msgs::PointCloud>();
            auto president_msg_ptr = std::make_shared<nav_msgs::Path>();
            make_msg(candidate_path_ptr, president_path_ptr, candidate_msg_ptr, president_msg_ptr);
            president_pub.publish(*president_msg_ptr);
            candidate_pub.publish(*candidate_msg_ptr);
            std::cout<<"================"<<std::endl;
        }

        if(goal_received){
            std::cout<<"goal_recieved"<<std::endl;
            pt_feedback.feedback.progress_percentage = progress;
            planning_tracking_as.publishFeedback(pt_feedback.feedback);

            pt_result.result.success = false;

            if(progress > 0.95){
                pt_result.result.success = true;
                planning_tracking_as.setSucceeded(pt_result.result);
                goal_received = false;
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    
}

void executeCB(const morai_woowa::Planning_Tracking_ActGoalConstPtr &goal){
    load_gpath(goal->path, goal->reverse);
    goal_received = true;
    pt_result.result.success = false;
    // planning_tracking_as.setSucceeded(pt_result.result); 
}   

void load_gpath(std::string path_name, bool is_reverse){
    global_path_ptr->clear();
    
    auto file_name = "/home/user/catkin_ws/src/Morai_Woowa/path/" + path_name;
    std::ifstream file(file_name);
    if (!file.is_open()) {
        ROS_ERROR("Failed to open file.");
        std::cout<<file_name<<std::endl;
        return;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string x_str, y_str, heading_str, index_str;

        // Read x, y, heading, and index from the CSV file
        std::getline(ss, x_str, ',');
        std::getline(ss, y_str, ',');
        std::getline(ss, heading_str, ',');
        std::getline(ss, index_str, ',');

        // 디버그용 출력: 읽은 값을 출력하여 확인
        // ROS_INFO("Read values: x_str=%s, y_str=%s", x_str.c_str(), y_str.c_str());

        // Check if x and y values are not empty
        if (x_str.empty() || y_str.empty()) {
            ROS_WARN("Empty x or y value, skipping line");
            continue;  // Skip this line if x or y is empty
        }

        try {
            double x = std::stof(x_str);
            double y = std::stof(y_str);

            global_path_ptr->push_back({x, y, 0.0});

        } catch (const std::invalid_argument& e) {
            ROS_WARN("Invalid data encountered: %s", e.what());
            continue;  // Skip this line if there's an invalid value
        } catch (const std::out_of_range& e) {
            ROS_WARN("Out of range error: %s", e.what());
            continue;  // Skip this line if values are out of range
        }
    }

    file.close();

    if(is_reverse){
        std::reverse(global_path_ptr->begin(), global_path_ptr->end());
    }
}

void obstacle_callback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    pcl::fromROSMsg(*msg, *cloud_ptr);
}

// void gpath_callback(const nav_msgs::Path::ConstPtr& msg) {
//     global_path_ptr->clear();
//     global_path_ptr->reserve(msg->poses.size());

//     for (int i = 0; i < msg->poses.size(); i++) {
//         double x = msg->poses[i].pose.position.x;
//         double y = msg->poses[i].pose.position.y;
//         global_path_ptr->push_back({x, y, 0});  // x, y를 vector로 저장
//     }
// }

void gpath_cut(){
    int idx = -1;
    int closet_idx = -1;
    double closet_dis = 9999;
    for (int i=0; i< global_path_ptr->size(); i++){
        auto g_x = global_path_ptr->at(global_path_ptr->size() - i -1).at(0);
        auto g_y = global_path_ptr->at(global_path_ptr->size() - i -1).at(1);
        auto dis = calculateDistance({g_x, g_y, 0.0}, {pose_ptr->at(0), pose_ptr->at(1), 0.0});
        // if(dis < 0.5){
        //     idx = global_path_ptr->size() - i -1;
        //     break;
        // }
        if(dis < closet_dis){
            closet_dis = dis;
            closet_idx = global_path_ptr->size() - i -1;
        }
    }

    if(idx == -1){
        idx = closet_idx;
    }
    
    std::vector<std::vector<double>> sliced(global_path_ptr->begin()+idx, global_path_ptr->end());

    if(sliced.size() > predict_time * 10){
        sliced.resize(predict_time * 10);
    }

    progress = sliced.size() / global_path_ptr->size();

    *global_path_ptr = sliced;
}

void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    pose_ptr->clear();
    pose_ptr->resize(3);

    // Extract position
    pose_ptr->at(0) = msg->pose.position.x;
    pose_ptr->at(1) = msg->pose.position.y;

    // Extract orientation (quaternion)
    double qx = msg->pose.orientation.x;
    double qy = msg->pose.orientation.y;
    double qz = msg->pose.orientation.z;
    double qw = msg->pose.orientation.w;

    // Convert quaternion to yaw
    tf::Quaternion quat(qx, qy, qz, qw);
    tf::Matrix3x3 m(quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    pose_ptr->at(2) = yaw; 
}

void filter_obstacle(){
    std::vector<std::vector<double>> filtered_obstacle;
    filtered_obstacle.reserve(obstacle_ptr->size()); 
    for(int i=0; i < obstacle_ptr->size(); i++){
        std::vector<double> pose;
        pose = {pose_ptr->at(0), pose_ptr->at(1), 0.0};

        if(calculateDistance(obstacle_ptr->at(i), pose) < 5){
            filtered_obstacle.push_back(obstacle_ptr->at(i));
        }
    }
    *obstacle_ptr = filtered_obstacle;
}


void make_candidate_path(std::shared_ptr<std::vector<std::vector<std::vector<double>>>> candidate_path_ptr){
    std::cout<<"cadi_start"<<std::endl;
    auto av_gap = 2 * angular_velocity / (num_of_path - 1);
    candidate_path_ptr->clear();
    candidate_path_ptr->reserve((num_of_path + 1));

    candidate_path_ptr->push_back(*global_path_ptr);

    for(int i = 0; i < num_of_path; i++){
        std::vector<std::vector<double>> candidate_i;
        candidate_i.reserve(global_path_ptr->size());
        auto current_angular_velocity = angular_velocity - av_gap * i ;
        for(int j = 0; j < global_path_ptr->size(); j++){
            auto time = j * 0.1;
            std::vector<double> candidate_element = {pose_ptr->at(0) + velocity * time * std::cos(current_angular_velocity * time + pose_ptr->at(2)), pose_ptr->at(1) + velocity * time * std::sin(current_angular_velocity * time + pose_ptr->at(2)), 0.0};
            candidate_i.push_back(candidate_element); 
        }
        candidate_path_ptr->push_back(candidate_i); 
    }
    std::cout<<"cadi_done"<<std::endl;
}

void vote_president_path(std::shared_ptr<std::vector<std::vector<std::vector<double>>>> candidate_path_ptr){
    std::cout<<"vote_start"<<std::endl;
    auto best_cost = 999999.9;
    obstacle_ptr->clear();
    obstacle_ptr->resize(cloud_ptr->size());

    for(int i =0; i < num_of_path + 1; i++){
        double current_cost = 0.0;
        auto current_candidate = candidate_path_ptr->at(i);
        for(int j = 0; j < current_candidate.size(); j++){
            // obstacle cost
            for( int k = 0 ; k < cloud_ptr->size(); k++){
                obstacle_ptr->at(k).resize(3);
                obstacle_ptr->at(k).at(0) = cloud_ptr->at(k).x + cloud_ptr->at(k).z * j * 0.1;
                obstacle_ptr->at(k).at(1) = cloud_ptr->at(k).y + cloud_ptr->at(k).intensity * j * 0.1;
                obstacle_ptr->at(k).at(2) = 0.0;

                auto distance = calculateDistance(current_candidate.at(j), obstacle_ptr->at(k));
                if(distance < 1.5){
                    current_cost += 9999;
                }
                // current_cost += obstacle_cost / std::pow(distance, 2);
            }

            // path cost
            auto distance = calculateDistance(current_candidate.at(j), global_path_ptr->at(j));
            current_cost += global_path_cost * std::pow(distance, 1);
            }



        std::cout<<i<<" cost: "<<current_cost<<std::endl;

        if(i == 0 & current_cost < 9999){
            best_idx = i;
            best_cost = current_cost;
            break;
        }

        if(current_cost < best_cost){
            best_idx = i;
            best_cost = current_cost;
        }
    }
    president_path_ptr->clear();
    for(int j = 0; j < global_path_ptr->size(); j++){
        president_path_ptr->push_back(candidate_path_ptr->at(best_idx).at(j));
    }

    std::cout<<"vote_done"<<std::endl;

}

void make_msg(std::shared_ptr<std::vector<std::vector<std::vector<double>>>> candidate_path_ptr, 
             std::shared_ptr<std::vector<std::vector<double>>> president_path_ptr,
             std::shared_ptr<sensor_msgs::PointCloud> candidate_msg_ptr, std::shared_ptr<nav_msgs::Path> president_msg_ptr){

    // 후보 경로 (candidate_path_ptr) 처리
    for(int i = 0; i < candidate_path_ptr->size(); i++){
        auto candidate_i = candidate_path_ptr->at(i);
        for(int j = 0; j < candidate_i.size(); j++){
            auto element = candidate_i.at(j);
            geometry_msgs::Point32 point;
            point.x = element.at(0);
            point.y = element.at(1);
            point.z = 0.0;
            candidate_msg_ptr->points.push_back(point);
        }
    }
    candidate_msg_ptr->header.frame_id = "map";

    // 대통령 경로 (president_path_ptr) 처리, nav_msgs::Path로 변환
    for(int j = 0; j < president_path_ptr->size(); j++){
        auto element = president_path_ptr->at(j);
        
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = "map";  // frame_id 설정
        pose_stamped.header.stamp = ros::Time::now();  // 현재 시간

        // 위치 설정
        pose_stamped.pose.position.x = element.at(0);
        pose_stamped.pose.position.y = element.at(1);
        pose_stamped.pose.position.z = 0.0;

        // 방향 (orientation)은 기본적으로 단위 쿼터니언 사용
        pose_stamped.pose.orientation.x = 0.0;
        pose_stamped.pose.orientation.y = 0.0;
        pose_stamped.pose.orientation.z = 0.0;
        pose_stamped.pose.orientation.w = 1.0;

        // 대통령 경로에 추가
        president_msg_ptr->poses.push_back(pose_stamped);
    }

    // president_msg_ptr의 헤더 설정
    president_msg_ptr->header.frame_id = "map";
    president_msg_ptr->header.stamp = ros::Time::now();
}

double calculateDistance(const std::vector<double>& point1, const std::vector<double>& point2) {
    double distance = std::sqrt(
        std::pow(point2[0] - point1[0], 2) +
        std::pow(point2[1] - point1[1], 2) +
        std::pow(point2[2] - point1[2], 2)
    );

    return distance;
}