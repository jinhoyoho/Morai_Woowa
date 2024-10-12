/*
#include <ros/ros.h>
#include <ros/package.h>  
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include "morai_msgs/WoowaDillyEventCmdSrv.h"
#include "morai_msgs/DillyCmd.h"
#include "morai_msgs/DillyCmdResponse.h"
#include "Morai_Woowa/way_point.h"
#include <tf/transform_datatypes.h>

#include <vector>
#include <fstream>
#include <sstream>
#include <cmath>
#include <limits>
#include <string>
#include <filesystem>
#include <thread>  // 스레드 사용을 위한 헤더

typedef std::vector<Waypoint> path;

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
        waypoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/waypoints", 10);  // PoseStamped 퍼블리셔
        waypoint_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/globalpath", 10, this);

        loadWaypoints(waypoint_file_1_, waypoints_1_);  // waypoint 파일에서 로드
        loadWaypoints(waypoint_file_2_, waypoints_2_);  // waypoint 파일에서 로드
        loadWaypoints(waypoint_file_3_, waypoints_3_);  // waypoint 파일에서 로드
    }

    void currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {

        closest_index_ = findClosestWaypoint(msg->pose.position.x, msg->pose.position.y, closest_index_, waypoints_1_);
        
        if (closest_index_ != -1) {
            ROS_INFO("Current Position: (X: %.2f, Y: %.2f) is nearest to Waypoint Index: %d", msg->pose.position.x, msg->pose.position.y, closest_index_);
            ROS_INFO("cross_track_error: %lf", cross_track_error);
        } else {
            ROS_WARN("No waypoints available.");
        }
    }

    void pub_waypoints() {
        // CSV에서 읽은 웨이포인트를 퍼블리시
        PublishWaypoints(waypoints_1_);
        PublishWaypoints(waypoints_2_);
        PublishWaypoints(waypoints_3_);

        PublishMarker(waypoints_1_);
        PublishMarker(waypoints_2_);
        PublishMarker(waypoints_3_);
    }

    void PublishWaypoints(path waypoints) {
        for (const auto& waypoint : waypoints) {
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header.frame_id = "map";
            pose_stamped.header.stamp = ros::Time::now();

            // Waypoint 좌표 설정
            pose_stamped.pose.position.x = waypoint.x;
            pose_stamped.pose.position.y = waypoint.y;
            pose_stamped.pose.position.z = 0;

            // Heading 값을 그대로 사용하여 orientation 설정
            tf::Quaternion q = tf::createQuaternionFromYaw(waypoint.heading); // 여전히 쿼터니언 생성이 필요할 경우
            pose_stamped.pose.orientation.x = q.x();
            pose_stamped.pose.orientation.y = q.y();
            pose_stamped.pose.orientation.z = q.z();
            pose_stamped.pose.orientation.w = q.w();

            // PoseStamped 퍼블리시
            waypoint_pub_.publish(pose_stamped);

        }
    }

    void PublishMarker(path waypoints) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "waypoints";
        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::SPHERE;
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
            ROS_INFO("closest_index: %d", closest_index_);

            pub_waypoints();
            rate.sleep();  // 지정된 주기로 대기
        }
    }
    
private:
    ros::NodeHandle nh_;
    ros::Subscriber current_pose_sub_;
    ros::Publisher waypoint_pub_; // PoseStamped 퍼블리셔
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

    void PlanningActionDoneCb(const actionlib::SimpleClientGoalState& state,
                              const morai_woowa::Planning_Tracking_ActResultConstPtr& result){
        
        is_planning_action_finished_ = true;

        if (result->success){
            is_reach_target_point = true;
            std::cout << "state node : planning action finished successfully" << std::endl;
        } 
        else {
            is_reach_target_point = false;
            std::cout << "state node : Action did not finish successfully" << std::endl;
        }
    }

    void PlanningActionActiveCb(){
        std::cout << "state_node : planning action is active." << std::endl;
    }

    void PlanningActionFeedbackCb(const morai_woowa::Planning_Tracking_ActFeedbackConstPtr& feedback){
        std::cout << "state node : planner action feedback: " << feedback->progress_percentage << std::endl;
    }

    void PersonCollisionActionDoneCb(const actionlib::SimpleClientGoalState& state,
                              const morai_woowa::Person_Collision_ActResultConstPtr& result){
        
        is_collision_action_finished_ = true;

        if (result->success){
            is_success_collision = true;
            std::cout << "state node : collision action finished successfully" << std::endl;
        } 
        else {
            is_success_collision = false;
            std::cout << "state node : collision Action did not finish successfully" << std::endl;
        }    
    }

    void PersonCollisionActionActiveCb(){
        std::cout << "state_node : collision action is active." << std::endl;
    }

    void PersonCollisionActionFeedbackCb(const morai_woowa::Person_Collision_ActFeedbackConstPtr& feedback){
        std::cout << "state node : planner action feedback: " << feedback->is_target << std::endl;
    }

    bool set_target_point(int a, int b, bool indoor){

        std::ostringstream oss;
        std::string filename;
        
        if (indoor) {
            oss << "indoor_" << a << "_" << b << ".csv";
            filename = oss.str();
        }
        else{
            oss << "outdoor_" << a << "_" << b << ".csv";
            filename = oss.str();
        }

        loadWaypoints(filename, waypoints_);

        morai_woowa::Planning_Tracking_ActGoal planning_goal;
        planning_goal.path = filename;

        planning_tracking_ac_.sendGoal(planning_goal,
                    boost::bind(&StateNode::PlanningActionDoneCb, this, boost::placeholders::_1, boost::placeholders::_2),
                    boost::bind(&StateNode::PlanningActionActiveCb, this),
                    boost::bind(&StateNode::PlanningActionFeedbackCb, this, boost::placeholders::_1));

        is_planning_action_finished_ = false;
        is_reach_target_point = false;
        ros::Rate loop_rate(0.5);

        while (!is_planning_action_finished_ && ros::ok()){
            loop_rate.sleep();
            ros::spinOnce();
        }

        if(is_reach_target_point)
            return true; 
        else
            return false;
    }

    bool collision_to_person(float range){

        morai_woowa::Person_Collision_ActGoal collision_goal;
        collision_goal.range = range; 

        person_collision_ac_.sendGoal(collision_goal,
                    boost::bind(&StateNode::PersonCollisionActionDoneCb, this, boost::placeholders::_1, boost::placeholders::_2),
                    boost::bind(&StateNode::PersonCollisionActionActiveCb, this),
                    boost::bind(&StateNode::PersonCollisionActionFeedbackCb, this, boost::placeholders::_1));

        is_collision_action_finished_ = false;
        is_success_collision = false;
        ros::Rate loop_rate(0.5);

        while (!is_collision_action_finished_ && ros::ok()){
            loop_rate.sleep();
            ros::spinOnce();
        }

        if(is_success_collision)
            return true; 
        else
            return false;
    }


    // 여기가 메인 state 함수임!!
    void state() {
        ros::Rate rate(100);  // 0.01 Hz
        while (ros::ok()) {

            set_target_point(0,1,true);

            collision_to_person(10);

            publishWaypoints(waypoints_);
            
            rate.sleep();  // 지정된 주기로 대기
        }
    }

private:
    ros::NodeHandle nh_;

    actionlib::SimpleActionClient<morai_woowa::Planning_Tracking_ActAction> planning_tracking_ac_;
    actionlib::SimpleActionClient<morai_woowa::Person_Collision_ActAction> person_collision_ac_;

    ros::Subscriber current_pose_sub_;
    ros::Publisher waypoint_marker_pub_;

    ros::ServiceClient delivery_pickup_client_;

    bool is_planning_action_finished_;
    bool is_reach_target_point;

    bool is_collision_action_finished_;
    bool is_success_collision;

    path waypoints_;

    int closest_index_ = 0;
    double cross_track_error = 10000000; 

    bool wpt_init_flag = false;
    
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "state_node");

    StateNode StateNode;

    std::thread thread(&StateNode::state, &StateNode);

    ros::spin();
    
    thread.join();  // 스레드가 종료될 때까지 대기

    return 0;
}*/