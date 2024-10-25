#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
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
            gpath_pub = nh.advertise<nav_msgs::Path>("gpath", 10);
            progress_pub = nh.advertise<std_msgs::Float32>("progress", 10);

            obstacle_sub = nh.subscribe("obstacle", 10, &Dwa::obstacle_callback, this);
            pose_sub = nh.subscribe("current_pose", 10, &Dwa::pose_callback, this);
            mode_sub = nh.subscribe("mode", 10, &Dwa::mode_callback, this);

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

            cout<<"============="<<"\n";

            rate.sleep();
        }

        // 성공했을 때 progress 0 publish해서 멈추도록
        std_msgs::Float32 progress_msg;
        feedback.feedback.progress_percentage = 0;
        progress_msg.data = feedback.feedback.progress_percentage;
        progress_pub.publish(progress_msg);
    }

    void obstacle_callback(const sensor_msgs::PointCloud2::ConstPtr& msg){
        pcl::fromROSMsg(*msg, *cloud_ptr);
    }

    void mode_callback(const std_msgs::Int8::ConstPtr& msg)
    {
        mode = msg->data;
    }

    void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
        pose_ptr->clear();
        pose_ptr->resize(3);

        // Extract position
        (*pose_ptr)[0] = msg->pose.position.x;
        (*pose_ptr)[1] = msg->pose.position.y;

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

        (*pose_ptr)[2] = yaw; 
    }

    void load_path(std::string path_name, bool is_reverse){
        global_path_ptr->clear();

        gpath_name = path_name;

        // if(path_name == "outdoor_7_1.csv" || path_name == "outdoor_6_2.csv"){
        //     signal_path = true;
        // }
        // else{
        //     signal_path = false;
        // }

        auto file_name = ros::package::getPath("morai_woowa") + "/path/" + path_name;
        // auto file_name = "/home/user/catkin_ws/src/Morai_Woowa/path/" + path_name;

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
            auto k = (*global_path_ptr)[global_path_ptr->size() - i -1];
            auto g_x = (*global_path_ptr)[global_path_ptr->size() - i -1][0];
            auto g_y = (*global_path_ptr)[global_path_ptr->size() - i -1][1];
            auto dis = calculateDistance({g_x, g_y, 0.0}, {(*pose_ptr)[0] , (*pose_ptr)[1] , 0.0});
            if(dis < closet_dis){
                closet_dis = dis;
                closet_idx = global_path_ptr->size() - i -1;
            }
        }

        path_normal_distance = closet_dis;

        if(idx == -1){
            idx = closet_idx;
        }
        
        std::vector<std::vector<double>> sliced(global_path_ptr->begin()+closet_idx, global_path_ptr->end());

        if(sliced.size() > predict_time * 10){
            sliced.resize(predict_time * 10);
        }

        
        auto progress = calculateDistance({(*global_path_ptr).back()[0], (*global_path_ptr).back()[1], 0.0}, {(*pose_ptr)[0] , (*pose_ptr)[1] , 0.0});
        
        // std::cout<<"gpp"<<(*global_path_ptr).back()[0]<<"\n";
        // std::cout<<"gpp"<<(*global_path_ptr).back()[1]<<"\n";
        // std::cout<<"pose"<<(*pose_ptr)[0]<<"\n";
        // std::cout<<"pose"<<(*pose_ptr)[1]<<"\n";
        feedback.feedback.progress_percentage = progress;

        cout<<"progress"<<feedback.feedback.progress_percentage<<endl;

        *remain_global_path_ptr = sliced;
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

        candidate_path_ptr->push_back(*remain_global_path_ptr);

        for(int i = 0; i < num_of_path; i++){

            std::vector<std::vector<double>> candidate_i;
            candidate_i.reserve(remain_global_path_ptr->size());
            auto current_angular_velocity = angular_velocity - av_gap * i ;
            for(int j = 0; j < remain_global_path_ptr->size(); j++){
                auto time = j * 0.1;
                std::vector<double> candidate_element = {(*pose_ptr)[0] + velocity * time * std::cos(current_angular_velocity * time + (*pose_ptr)[2] ), (*pose_ptr)[1]  + velocity * time * std::sin(current_angular_velocity * time + (*pose_ptr)[2] ), 0.0};
                candidate_i.push_back(candidate_element); 
            }
            candidate_path_ptr->push_back(candidate_i); 
        }
    }

    void vote_president_path(){
        auto best_cost = 999999.9;
        auto best_idx = 0;
        std::string direction_flag = "none" ;

        //신호등에 있을땐 모든 장애물을 없앤다. 봉에 박지 않게 하기 위해서
        if(gpath_name == "outdoor_7_1.csv" && (feedback.feedback.progress_percentage < 35 && feedback.feedback.progress_percentage > 9)){
            cloud_ptr->clear();
        }

        else if (gpath_name == "outdoor_6_2.csv" && ((feedback.feedback.progress_percentage < 64 || feedback.feedback.progress_percentage > 36)
                || (feedback.feedback.progress_percentage < 140 && feedback.feedback.progress_percentage > 132)) ){
            cloud_ptr->clear();
        }

        std::cout<<"cp"<<cloud_ptr->size()<<"\n";

        //obstacle이 하나도 없을때
        if(!cloud_ptr){
            president_path_ptr->clear();
            for(int j = 0; j < remain_global_path_ptr->size(); j++){
                president_path_ptr->push_back((*candidate_path_ptr)[0][j]);
            }
        }

        else{
            future_obstacle_ptr->clear();
            future_obstacle_ptr->resize(cloud_ptr->size());

            for(int path_idx =0; path_idx < num_of_path + 1; path_idx++){
                double current_cost = 0.0;
                auto current_candidate = (*candidate_path_ptr)[path_idx];

                for(int current_time_idx = 0; current_time_idx < current_candidate.size(); current_time_idx++){
                    auto current_time = current_time_idx * 0.1;
                    // obstacle cost 모든 장애물에 대해 미래 위치를 계산한다.
                    for( int k = 0 ; k < cloud_ptr->size(); k++){
                        (*future_obstacle_ptr)[k].resize(3);
                        //z는 x축 속도정보, intensity는 y축 속도정보
                        (*future_obstacle_ptr)[k][0] = (*cloud_ptr)[k].x + (*cloud_ptr)[k].z * current_time;
                        (*future_obstacle_ptr)[k][1] = (*cloud_ptr)[k].y + (*cloud_ptr)[k].intensity * current_time;
                        (*future_obstacle_ptr)[k][2] = 0.0;

                        auto distance = calculateDistance(current_candidate[current_time_idx], (*future_obstacle_ptr)[k]);
                        if(distance < 1.0){
                            current_cost += 9999;
                        }

                        // 전역경로 가운데에 장애물이 있을 경우에
                        if(distance < 1.0 && path_idx == 0 && (*cloud_ptr)[k].z < 0.1 && (*cloud_ptr)[k].intensity < 0.1 && direction_flag == "none"){
                            std::vector<double> obstacle_vector = {(*cloud_ptr)[k].x - (*remain_global_path_ptr)[0][0], (*cloud_ptr)[k].y - (*remain_global_path_ptr)[0][1]};
                            std::vector<double> path_vector = {(*remain_global_path_ptr).back()[0] - (*remain_global_path_ptr)[0][0], (*remain_global_path_ptr).back()[1] - (*remain_global_path_ptr)[0][1]};
                            auto direction = obstacle_vector[0] * path_vector[1] - obstacle_vector[1] * path_vector[0];

                            if(direction > 0){
                                direction_flag = "left";
                            }
                            else{
                                direction_flag = "right";
                            }

                            std::cout<<"direction_flag: "<<direction_flag<<"\n";
                        }
                    }

                    // path cost
                    auto distance = calculateDistance(current_candidate[current_time_idx], (*remain_global_path_ptr)[current_time_idx]);
                    current_cost += global_path_cost * std::pow(distance, 3);
                    }

                if(path_idx > num_of_path/2 && direction_flag == "left"){
                    current_cost += 99999;
                }
                else if(path_idx < num_of_path/2 && direction_flag == "right"){
                    current_cost += 99999;
                }

                std::cout<<path_idx<<" cost: "<<current_cost<<std::endl;

                if(path_idx == 0 && current_cost < 9999 && path_normal_distance < 0.3){
                    best_idx = path_idx;
                    best_cost = current_cost;
                    break;
                }
                else if(path_idx == 0){
                    current_cost = 9999999;
                }

                if(current_cost < best_cost){
                    best_idx = path_idx;
                    best_cost = current_cost;
                }

            }

            if(best_cost > 9999){
                //가장 낮은 cost도 9999 넘으면 멈춤
                stop_flag = true;
            }

            //best path를 president에 복사
            president_path_ptr->clear();
            for(int j = 0; j < remain_global_path_ptr->size(); j++){
                president_path_ptr->push_back((*candidate_path_ptr)[best_idx][j]);
            }
        }

    }

    void msgs_publish(){
        //초기화
        lpath_msg_ptr->poses.clear();
        gpath_msg_ptr->poses.clear();
        candidate_msg_ptr->points.clear(); 

        // 후보 경로 (candidate_path_ptr) 처리
        for(int i = 0; i < candidate_path_ptr->size(); i++){
            auto candidate_i = (*candidate_path_ptr)[i];
            for(int j = 0; j < candidate_i.size(); j++){
                auto element = candidate_i[j];
                geometry_msgs::Point32 point;
                point.x = element[0];
                point.y = element[1];
                point.z = 0.0;
                candidate_msg_ptr->points.push_back(point);
            }
        }
        candidate_msg_ptr->header.frame_id = "map";

        //  nav_msgs::Path로 변환
        for(int j = 0; j < president_path_ptr->size(); j++){
            auto element = (*president_path_ptr)[j];
            
            
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header.frame_id = "map";  // frame_id 설정
            pose_stamped.header.stamp = ros::Time::now();  // 현재 시간

            // 위치 설정
            pose_stamped.pose.position.x = element[0];
            pose_stamped.pose.position.y = element[1];
            pose_stamped.pose.position.z = 0.0;

            // 방향 (orientation)은 기본적으로 단위 쿼터니언 사용
            pose_stamped.pose.orientation.x = 0.0;
            pose_stamped.pose.orientation.y = 0.0;
            pose_stamped.pose.orientation.z = 0.0;
            pose_stamped.pose.orientation.w = 1.0;

            // 대통령 경로에 추가
            lpath_msg_ptr->poses.push_back(pose_stamped);
        }

        for(int j = 0; j < global_path_ptr->size(); j++){
            auto element = (*global_path_ptr)[j];
            
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header.frame_id = "map";  // frame_id 설정
            pose_stamped.header.stamp = ros::Time::now();  // 현재 시간

            // 위치 설정
            pose_stamped.pose.position.x = element[0];
            pose_stamped.pose.position.y = element[1];
            pose_stamped.pose.position.z = 0.0;

            // 방향 (orientation)은 기본적으로 단위 쿼터니언 사용
            pose_stamped.pose.orientation.x = 0.0;
            pose_stamped.pose.orientation.y = 0.0;
            pose_stamped.pose.orientation.z = 0.0;
            pose_stamped.pose.orientation.w = 1.0;

            // 대통령 경로에 추가
            gpath_msg_ptr->poses.push_back(pose_stamped);
        }

        // president_msg_ptr의 헤더 설정
        lpath_msg_ptr->header.frame_id = "map";
        lpath_msg_ptr->header.stamp = ros::Time::now();

        gpath_msg_ptr->header.frame_id = "map";
        gpath_msg_ptr->header.stamp = ros::Time::now();
        
        lpath_pub.publish(*lpath_msg_ptr);
        gpath_pub.publish(*gpath_msg_ptr);
        candidate_pub.publish(*candidate_msg_ptr);
    }

    void feedback_publish(){
        as.publishFeedback(feedback.feedback);

        std_msgs::Float32 progress_msg;
        progress_msg.data = feedback.feedback.progress_percentage;
        progress_pub.publish(progress_msg);

        // 99% 이상이거나 90% 이상이고 갑자기 거리가 멀어지면 순간이동 성공으로 판단해 성공 반환
        // 사람 박치기 모드여야 성공
        if(feedback.feedback.progress_percentage < 0.2 
            || ((feedback.feedback.progress_percentage > 0.5) && (path_normal_distance > 300.0))){

            ROS_INFO("DWA SUCCESS!");
            
            result.result.success = true;
            stop_flag = true;

            // 성공반환할때 progress를 0으로 바꿈
            feedback.feedback.progress_percentage = 0;
            progress_msg.data = feedback.feedback.progress_percentage;
            progress_pub.publish(progress_msg);

            as.setSucceeded(result.result);
        }

        if(stop_flag == true){
            //progress를 0으로 바꿔서 멈추게 함
            feedback.feedback.progress_percentage = 0;
            progress_msg.data = feedback.feedback.progress_percentage;
            progress_pub.publish(progress_msg);
            stop_flag = false;
        }
    }


private:
    ros::NodeHandle nh;
    ros::Publisher candidate_pub;
    ros::Publisher lpath_pub;
    ros::Subscriber obstacle_sub;
    ros::Publisher gpath_pub;
    ros::Publisher progress_pub;
    ros::Subscriber pose_sub;
    ros::Subscriber mode_sub;
    int mode;
    
    int num_of_path = 15;
    double predict_time = 3.0;
    double velocity = 1.0;
    double angular_velocity = 0.5;
    double global_path_cost = 0.1;
    double path_normal_distance = 0.0; //path와 현재 위치 수직거리
    bool stop_flag = false;
    bool signal_path = false;
    std::string gpath_name;

    actionlib::SimpleActionServer<morai_woowa::Planning_Tracking_ActAction> as;
    morai_woowa::Planning_Tracking_ActActionResult result;
    morai_woowa::Planning_Tracking_ActActionFeedback feedback;
    shared_ptr<vector<vector<double>>> global_path_ptr = make_shared<vector<vector<double>>>();
    shared_ptr<vector<vector<double>>> remain_global_path_ptr = make_shared<vector<vector<double>>>();
    shared_ptr<vector<vector<vector<double>>>> candidate_path_ptr = make_shared<vector<vector<vector<double>>>>();
    shared_ptr<vector<vector<double>>> president_path_ptr = make_shared<vector<vector<double>>>();
    shared_ptr<vector<double>> pose_ptr = make_shared<vector<double>>(3);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    shared_ptr<vector<vector<double>>> future_obstacle_ptr = make_shared<vector<vector<double>>>();
    shared_ptr<sensor_msgs::PointCloud> candidate_msg_ptr = make_shared<sensor_msgs::PointCloud>();
    shared_ptr<nav_msgs::Path> lpath_msg_ptr = make_shared<nav_msgs::Path>();
    shared_ptr<nav_msgs::Path> gpath_msg_ptr = make_shared<nav_msgs::Path>();
};

int main(int argc, char** argv){
    ros::init(argc, argv, "dwa_node");

    Dwa Dwa;

    ros::spin();

    return 0;
}