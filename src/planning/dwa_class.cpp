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
#include <algorithm> 

using namespace std;

class Dwa{
public:
    Dwa()
        :as(nh, "planning_tracking", boost::bind(&Dwa::executeAction, this, _1), false){
            
            candidate_pub = nh.advertise<sensor_msgs::PointCloud>("/candidate_path", 10);
            lpath_pub = nh.advertise<nav_msgs::Path>("lpath", 10);

            obstacle_sub = nh.subscribe("obstacle", 10, &Dwa::obstacle_callback, this);
            pose_sub = nh.subscribe("current_pose", 10, &Dwa::pose_callback, this);

            as.start();
        }

    void executeAction(const morai_woowa::Planning_Tracking_ActGoalConstPtr &goal){
        ros::Rate rate(10);
        //goal 형식으로 된 path 불러오기
        load_path(goal->path, goal->reverse); 

        result.result.success = false;

        //성공할 때 까지 반복
        while(result.result.success == false){

            ros::spinOnce();

            //내 위치 기준으로 앞으로 갈 path만 자르기
            gpath_cut();

            //후보경로 생성
            make_candidate_path();

            //cost 계산하기
            vote_president_path();

            //msg형식으로 만들어서 publish
            msgs_publish();

            //progress 계산 후 action 처리
            feedback_publish();

            rate.sleep();
        }
    }

    void obstacle_callback(const sensor_msgs::PointCloud2::ConstPtr& msg){
        pcl::fromROSMsg(*msg, *cloud_ptr);
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

    void load_path(std::string path_name, bool is_reverse){
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
                // ROS_WARN("Invalid data encountered: %s", e.what());
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

    void gpath_cut(){
        int idx = -1;
        int closet_idx = -1;
        double closet_dis = 9999;
        for (int i=0; i< global_path_ptr->size(); i++){
            auto g_x = global_path_ptr->at(global_path_ptr->size() - i -1).at(0);
            auto g_y = global_path_ptr->at(global_path_ptr->size() - i -1).at(1);
            auto dis = calculateDistance({g_x, g_y, 0.0}, {pose_ptr->at(0), pose_ptr->at(1), 0.0});
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

        feedback.feedback.progress_percentage = sliced.size() / global_path_ptr->size();

        *global_path_ptr = sliced;
    }

    double calculateDistance(const std::vector<double>& point1, const std::vector<double>& point2) {
        double distance = std::sqrt(
            std::pow(point2[0] - point1[0], 2) +
            std::pow(point2[1] - point1[1], 2) +
            std::pow(point2[2] - point1[2], 2)
        );

        return distance;
    }

    void make_candidate_path(){
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
    }

    void vote_president_path(){
        auto best_cost = 999999.9;
        auto best_idx = 0;


        //obstacle이 하나도 없을때
        if(!cloud_ptr){
            future_obstacle_ptr->clear();
            future_obstacle_ptr->resize(cloud_ptr->size());
            president_path_ptr->clear();
            for(int j = 0; j < global_path_ptr->size(); j++){
                president_path_ptr->push_back(candidate_path_ptr->at(0).at(j));
            }
        }

        else{
            for(int path_idx =0; path_idx < num_of_path + 1; path_idx++){
                double current_cost = 0.0;
                auto current_candidate = candidate_path_ptr->at(path_idx);

                for(int current_time_idx = 0; current_time_idx < current_candidate.size(); current_time_idx++){
                    auto current_time = current_time_idx * 0.1;
                    // obstacle cost 모든 장애물에 대해 미래 위치를 계산한다.
                    for( int k = 0 ; k < cloud_ptr->size(); k++){
                        future_obstacle_ptr->at(k).resize(3);
                        //z는 x축 속도정보, intensity는 y축 속도정보
                        future_obstacle_ptr->at(k).at(0) = cloud_ptr->at(k).x + cloud_ptr->at(k).z * current_time;
                        future_obstacle_ptr->at(k).at(1) = cloud_ptr->at(k).y + cloud_ptr->at(k).intensity * current_time;
                        future_obstacle_ptr->at(k).at(2) = 0.0;

                        auto distance = calculateDistance(current_candidate.at(current_time_idx), future_obstacle_ptr->at(k));
                        if(distance < 1.5){
                            current_cost += 9999;
                        }
                    }

                    // path cost
                    auto distance = calculateDistance(current_candidate.at(current_time_idx), global_path_ptr->at(current_time_idx));
                    current_cost += global_path_cost * std::pow(distance, 1);
                    }



                std::cout<<path_idx<<" cost: "<<current_cost<<std::endl;

                if(path_idx == 0 & current_cost < 9999){
                    best_idx = path_idx;
                    best_cost = current_cost;
                    break;
                }

                if(current_cost < best_cost){
                    best_idx = path_idx;
                    best_cost = current_cost;
                }
            }
            //best path를 president에 복사
            president_path_ptr->clear();
            for(int j = 0; j < global_path_ptr->size(); j++){
                president_path_ptr->push_back(candidate_path_ptr->at(best_idx).at(j));
            }
        }

    }

    void msgs_publish(){
        //초기화
        lpath_msg_ptr->poses.clear();
        candidate_msg_ptr->points.clear();

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

        //  nav_msgs::Path로 변환
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
            lpath_msg_ptr->poses.push_back(pose_stamped);
        }

        // president_msg_ptr의 헤더 설정
        lpath_msg_ptr->header.frame_id = "map";
        lpath_msg_ptr->header.stamp = ros::Time::now();

        lpath_pub.publish(*lpath_msg_ptr);
        candidate_pub.publish(*candidate_msg_ptr);
    }

    void feedback_publish(){
        as.publishFeedback(feedback.feedback);
        if(feedback.feedback.progress_percentage > 0.95){
            result.result.success = true;
            as.setSucceeded(result.result);
        }
    }


private:
    ros::NodeHandle nh;
    ros::Publisher candidate_pub;
    ros::Publisher lpath_pub;
    ros::Subscriber obstacle_sub;
    ros::Subscriber pose_sub;
    
    int num_of_path = 15;
    double predict_time = 3;
    double velocity = 1.0;
    double angular_velocity = 0.5;
    double global_path_cost = 0.1;

    actionlib::SimpleActionServer<morai_woowa::Planning_Tracking_ActAction> as;
    morai_woowa::Planning_Tracking_ActActionResult result;
    morai_woowa::Planning_Tracking_ActActionFeedback feedback;
    shared_ptr<vector<vector<double>>> global_path_ptr = make_shared<vector<vector<double>>>();
    shared_ptr<vector<vector<vector<double>>>> candidate_path_ptr = make_shared<vector<vector<vector<double>>>>();
    shared_ptr<vector<vector<double>>> president_path_ptr = make_shared<vector<vector<double>>>();
    shared_ptr<vector<double>> pose_ptr = make_shared<vector<double>>(3);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    shared_ptr<vector<vector<double>>> future_obstacle_ptr = make_shared<vector<vector<double>>>();
    shared_ptr<sensor_msgs::PointCloud> candidate_msg_ptr = make_shared<sensor_msgs::PointCloud>();
    shared_ptr<nav_msgs::Path> lpath_msg_ptr = make_shared<nav_msgs::Path>();
};

int main(int argc, char** argv){
    ros::init(argc, argv, "dwa");

    Dwa Dwa;

    ros::spin();

    return 0;
}