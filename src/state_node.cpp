#include <ros/ros.h>
#include <ros/package.h>  
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>

#include "morai_msgs/WoowaDillyEventCmdSrv.h"
#include "morai_msgs/WoowaDillyStatus.h"

#include <actionlib/client/simple_action_client.h>
#include "morai_woowa/Planning_Tracking_ActAction.h"
#include "morai_woowa/Person_Collision_ActAction.h"
#include "morai_woowa/StopTrackingSrv.h"

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

struct Spot {
    float x;
    float y;
    Spot(float x_coord, float y_coord) : x(x_coord), y(y_coord) {}
};

using path = std::vector<Waypoint>;

class StateNode{ 

public:
    StateNode(): 
    planning_tracking_ac_(nh_, "/planning_tracking_action", true),
    person_collision_ac_(nh_, "/Sensor_main/person_collision_action", true)
    {
        dilly_item_status_sub_ = nh_.subscribe("/WoowaDillyStatus", 10, &StateNode::itemstatusCallback, this);
        current_pose_sub_ = nh_.subscribe("/current_pose", 10, &StateNode::currentPoseCallback, this);
        waypoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/waypoints", 10);  

        delivery_pickup_client_ = nh_.serviceClient<morai_msgs::WoowaDillyEventCmdSrv>("/WoowaDillyEventCmd");
        stop_tracking_client_ = nh_.serviceClient<morai_woowa::StopTrackingSrv>("/StopTracking");

        if (planning_tracking_ac_.waitForServer(ros::Duration(5.0))) {  // 5초 대기
            ROS_INFO("Connected to planning_tracking server");
        } else {
            ROS_WARN("planning_tracking server not available!");
        }
        
        if (person_collision_ac_.waitForServer(ros::Duration(5.0))) {  
            ROS_INFO("Connected to person_collision server");
        } else {
            ROS_WARN("person_collision server not available!");
        }

        is_planning_action_finished_ = false;
        is_reach_target_point_ = false;
        progress_percentage_ = 0;

        is_found_collision_target_ = false;
        is_collision_action_finished_ = false;
        is_success_collision_ = false;

        wpt_init_flag_ = false;

        closest_index_ = -1;
        cross_track_error = 10000000; 
        current_x_ = 0;
        current_y_ = 0;

        int dilly_item_status_cnt_ = 0;

    }

    void itemstatusCallback(const morai_msgs::WoowaDillyStatus::ConstPtr& msg){
        dilly_item_status_list_ = msg->deliveryItem;
        int cnt = 0;
        //dilly_item_status_cnt_ = dilly_item_status_list_.size();
        for(auto i : dilly_item_status_list_){
            cnt++;
        }
        dilly_item_status_cnt_ = cnt;
    }

    void currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        
        current_x_ = msg->pose.position.x;
        current_y_ = msg->pose.position.y;

        closest_index_ = findClosestWaypoint(current_x_, current_y_, closest_index_, waypoints_);
        
        if (closest_index_ != -1) {
            ROS_INFO("Current Position: (X: %.2f, Y: %.2f) is nearest to Waypoint Index: %d", current_x_, current_x_, closest_index_);
            ROS_INFO("cross_track_error: %lf", cross_track_error);
        } else {
            ROS_WARN("No waypoints available.");
        }
    }

    // 현재 경로 pub
    void PublishWaypoints(path waypoints_) {
        for (const auto& waypoint : waypoints_) {
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

    // 배달 함수
    bool delivery(int item_index){
        
        morai_msgs::WoowaDillyEventCmdSrv deli_srv;
        deli_srv.request.request.isPickup = true; 
        deli_srv.request.request.deliveryItemIndex = item_index; 
        
        delivery_pickup_client_.call(deli_srv);
        
        // ros::Rate loop_rate(0.5);
        // int cnt = 0;

        // while (!deli_srv.response.response.result && ros::ok() && cnt < 5){
        //     std::cout << "state node : Failed to delivery. Retrying..." << std::endl;
        //     delivery_pickup_client_.call(deli_srv);
        //     loop_rate.sleep();
        //     ros::spinOnce();
        //     cnt ++;
        // }

        return deli_srv.response.response.result;
    } 

    // 픽업 함수
    bool pickup(int item_index){

        morai_msgs::WoowaDillyEventCmdSrv pick_srv;
        pick_srv.request.request.deliveryItemIndex = item_index;
        pick_srv.request.request.isPickup = true;

        delivery_pickup_client_.call(pick_srv);

        return pick_srv.response.response.result;
    }

    // 파일이름으로 Waypoints load
    void loadWaypoints(std::string waypoint_file, path &waypoints) {

        wpt_init_flag_ = false;
        int cnt = 0;

        std::ifstream file(waypoint_file);
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
            ROS_INFO("Loaded waypoints from %s", waypoint_file.c_str());
        } else {
            ROS_WARN("Unable to open waypoint file: %s", waypoint_file.c_str());
        }
    }

    // 현재위치에서 가장 가까운 경로 idx 
    int findClosestWaypoint(float x, float y, int previous_index, path waypoints) {

        // 처음엔 모든 wpt를 돌면서 위치 체크
        if(!wpt_init_flag_){
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
            wpt_init_flag_ = true;

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
            is_reach_target_point_ = true;
            std::cout << "state node : planning action finished successfully" << std::endl;
        } 
        else {
            is_reach_target_point_ = false;
            std::cout << "state node : Action did not finish successfully" << std::endl;
        }
    }

    void PlanningActionActiveCb(){
        std::cout << "state_node : planning action is active." << std::endl;
    }

    void PlanningActionFeedbackCb(const morai_woowa::Planning_Tracking_ActFeedbackConstPtr& feedback){
        std::cout << "state node : planner action feedback: " << feedback->progress_percentage << std::endl;
        progress_percentage_ = feedback->progress_percentage;
    }

    void PersonCollisionActionDoneCb(const actionlib::SimpleClientGoalState& state,
                              const morai_woowa::Person_Collision_ActResultConstPtr& result){
        
        is_collision_action_finished_ = true;

        if (result->success){
            is_success_collision_ = true;
            std::cout << "state node : collision action finished successfully" << std::endl;
        } 
        else {
            is_success_collision_ = false;
            std::cout << "state node : collision Action did not finish successfully" << std::endl;
        }    
    }

    void PersonCollisionActionActiveCb(){
        std::cout << "state_node : collision action is active." << std::endl;
    }

    void PersonCollisionActionFeedbackCb(const morai_woowa::Person_Collision_ActFeedbackConstPtr& feedback){

        std::cout << "state node : planner action feedback: " << feedback->is_target << std::endl;
        if (feedback->is_target){
            is_found_collision_target_ = true;
        } 
        else {
            is_found_collision_target_ = false;
        } 
    }

    // 도착 실패시 0 반환
    // 도착 성공시 arrival_point 반환
    int request_planning(int starting_point, int arrival_point, bool indoor){

        std::ostringstream oss1;
        std::ostringstream oss2;
        std::string filename;
        std::string out_or_in;
        morai_woowa::Planning_Tracking_ActGoal planning_goal;

        if (indoor) 
            out_or_in = "indoor";
        else
            out_or_in = "outdoor";

        //파일 문자열 조합
        oss1 << out_or_in << "_" << starting_point << "_" << arrival_point << ".csv";
        filename = oss1.str();

        //파일 존재하면 그 경로로 reverse 모드 안하고 request
        std::ifstream file(filename);
        if (file.is_open()) {
            planning_goal.path = filename;  
            planning_goal.reverse = false;  
        }
        //파일 존재하지 않으면 경로 이름 뒤집고
        else {
            oss2 << out_or_in << "_" << arrival_point << "_" << starting_point << ".csv";
            filename = oss2.str();
            std::ifstream file(filename);

            // 뒤집었을 때 경로 존재하면 이 경로로 reverse모드로 request
            if (file.is_open()) {
                planning_goal.path = filename;
                planning_goal.reverse = true;  
            }
            // 이래도 파일 안열리면 걍 없는 경로임 오류 생성
            else {
                ROS_WARN("Unable to open waypoint file: %s", filename.c_str());
                return false;
            }
        }
        loadWaypoints(filename, waypoints_);

        // 경로 트래킹 요청
        planning_tracking_ac_.sendGoal(planning_goal,
                    boost::bind(&StateNode::PlanningActionDoneCb, this, boost::placeholders::_1, boost::placeholders::_2),
                    boost::bind(&StateNode::PlanningActionActiveCb, this),
                    boost::bind(&StateNode::PlanningActionFeedbackCb, this, boost::placeholders::_1));

        // 관련 멤버변수 초기화
        is_planning_action_finished_ = false;
        is_reach_target_point_ = false;
        progress_percentage_ = 0;

        ros::Rate loop_rate(0.5);

        while (!is_planning_action_finished_ && ros::ok()){
            loop_rate.sleep();
            ros::spinOnce();
        }

        if(is_reach_target_point_)
            return arrival_point; 
        else
            return 0;
    }

    // 도착 실패시 0 반환
    // 충돌 성공시 0 반환
    // 도착 성공시 arrival_point 반환
    int request_planning_with_collision(int starting_point, int arrival_point, bool indoor, float range){

        std::ostringstream oss1;
        std::ostringstream oss2;
        std::string filename;
        std::string out_or_in;
        morai_woowa::Planning_Tracking_ActGoal planning_goal;

        if (indoor) 
            out_or_in = "indoor";
        else
            out_or_in = "outdoor";
        
        //파일 문자열 조합
        oss1 << out_or_in << "_" << starting_point << "_" << arrival_point << ".csv";
        filename = oss1.str();

        std::ifstream file(filename);

        //파일 존재하면 그 경로로 reverse 모드 안하고 request
        if (file.is_open()) {
            planning_goal.path = filename;  
            planning_goal.reverse = false;  
        }
        //파일 존재하지 않으면 경로 이름 뒤집고
        else {
            oss2 << out_or_in << "_" << arrival_point << "_" << starting_point << ".csv";
            filename = oss2.str();
            std::ifstream file(filename);

            // 뒤집었을 때 경로 존재하면 이 경로로 reverse모드로 request
            if (file.is_open()) {
                planning_goal.path = filename;
                planning_goal.reverse = true;  
            }
            // 이래도 파일 안열리면 걍 없는 경로임 오류 생성
            else {
                ROS_WARN("Unable to open waypoint file: %s", filename.c_str());
                return false;
            }
        }
        loadWaypoints(filename, waypoints_);

        // 경로 트래킹 요청
        planning_tracking_ac_.sendGoal(planning_goal,
                    boost::bind(&StateNode::PlanningActionDoneCb, this, boost::placeholders::_1, boost::placeholders::_2),
                    boost::bind(&StateNode::PlanningActionActiveCb, this),
                    boost::bind(&StateNode::PlanningActionFeedbackCb, this, boost::placeholders::_1));

        // 관련 멤버변수 초기화
        is_planning_action_finished_ = false;
        is_reach_target_point_ = false;
        progress_percentage_ = 0; 

        // 사람 충돌 요청
        morai_woowa::Person_Collision_ActGoal collision_goal;
        collision_goal.range = range; 

        person_collision_ac_.sendGoal(collision_goal,
                    boost::bind(&StateNode::PersonCollisionActionDoneCb, this, boost::placeholders::_1, boost::placeholders::_2),
                    boost::bind(&StateNode::PersonCollisionActionActiveCb, this),
                    boost::bind(&StateNode::PersonCollisionActionFeedbackCb, this, boost::placeholders::_1));

        // 관련 멤버변수 초기화
        is_collision_action_finished_ = false;
        is_success_collision_ = false;
        is_found_collision_target_ = false;

        // 한번 사람충돌 실패하면 걍 원래경로 주행하도록하는 flag
        bool tracking_stop_flag = false;

        ros::Rate loop_rate(0.5);

        while (!is_planning_action_finished_ && ros::ok()){
            
            // 만약 전체 경로의 25프로도 덜왔을 때 충돌타겟을 발견하면
            if(is_found_collision_target_ && progress_percentage_ < 0.25 && !tracking_stop_flag){
                
                tracking_stop_flag = true; 

                //충돌타겟 발견시 일단 트래킹 프로세스는 멈춰
                morai_woowa::StopTrackingSrv stop_srv;
                stop_srv.request.stop = true;
                stop_tracking_client_.call(stop_srv);

                // 충돌에 집중
                while (!is_collision_action_finished_ && ros::ok()){
                    loop_rate.sleep();
                    ros::spinOnce();
                }
                
                // while문 탈출시는 충돌액션이 끝난거임

                // 충돌 성공시
                if(is_success_collision_){
                    planning_tracking_ac_.cancelGoal();
                    return 0;
                } 
                // 충돌 실패시
                else{
                    stop_srv.request.stop = false;
                    stop_tracking_client_.call(stop_srv);
                }
            }
            
            loop_rate.sleep();
            ros::spinOnce();
        }

        person_collision_ac_.cancelGoal();

        if(is_reach_target_point_)
            return arrival_point;
        else
            return 0;
    }

    bool request_collision_to_person(float range){

        morai_woowa::Person_Collision_ActGoal collision_goal;
        collision_goal.range = range; 

        person_collision_ac_.sendGoal(collision_goal,
                    boost::bind(&StateNode::PersonCollisionActionDoneCb, this, boost::placeholders::_1, boost::placeholders::_2),
                    boost::bind(&StateNode::PersonCollisionActionActiveCb, this),
                    boost::bind(&StateNode::PersonCollisionActionFeedbackCb, this, boost::placeholders::_1));

        is_collision_action_finished_ = false;
        is_success_collision_ = false;
        is_found_collision_target_ = false;

        ros::Rate loop_rate(0.5);

        while (!is_collision_action_finished_ && ros::ok()){
            loop_rate.sleep();
            ros::spinOnce();
        }

        if(is_success_collision_)
            return true; 
        else
            return false;
    }

    bool check_teleport_success(std::string type){

        float dis = 10000;
        
        if (type == "teloport_indoor"){ // 실내 펠레포트 지점
            Spot teloport_spot_indoor(3.00, -42.00);
            dis = calculateDistance(current_x_, current_y_, teloport_spot_indoor.x, teloport_spot_indoor.y);
        }
        else if (type == "teloport_outdoor"){ // 야외 텔레포트 지점
            Spot teloport_spot_outdoor(430.00, -140.00);
            dis = calculateDistance(current_x_, current_y_, teloport_spot_outdoor.x, teloport_spot_outdoor.y);        
        }
        else if (type == "respawn_indoor"){ // 실내 리스폰 지점
            Spot respawn_spot_indoor(430.00, -140.00);
            dis = calculateDistance(current_x_, current_y_, respawn_spot_indoor.x, respawn_spot_indoor.y);        
        }
        else if (type == "respawn_outdoor"){ // 야외 리스폰 지점
            Spot respawn_spot_outdoor(430.00, -140.00);
            dis = calculateDistance(current_x_, current_y_, respawn_spot_outdoor.x, respawn_spot_outdoor.y);        
        }
        else{
            ROS_WARN("teleport fail >> strange type : %s", type.c_str());
            return false;
        }    

        if (dis < 0.1){
            ROS_WARN("teleport success: %s", type.c_str());
            return true;
        }
        else{
            ROS_WARN("teleport fail: %s", type.c_str());
            return false;
        }
    }

    bool check_arrival_success(int point, bool indoor){

        float dis = 10000;
        Spot target_spot(-99999,-99999)
        
        if (indoor){ // 실내
            if(point == 1){    
                Spot target_spot(-42.4,  -135.0);
            }
            else if(point == 2){  
                Spot target_spot(51.5,    -41.3);
            }
            else if(point == 3){
                Spot target_spot(65.2,     44.1);
            }
            else if(point == 4){ 
                Spot target_spot(-77.7,      7.7);
            }
            else if(point == 5){ 
                Spot target_spot(30.7,   -41.1);
            }
            else{
                ROS_WARN("arrival fail >> strange type : %d", point);
                return false;
            }  
        }
        else{
            if(point == 1){    
                Spot target_spot(393.9,-79.5);
            }
            else if(point == 2){  
                Spot target_spot(263.4,-79.1);
            }
            else if(point == 3){
                Spot target_spot(229.8,-129.1);
            }
            else if(point == 4){ 
                Spot target_spot(334.9,-117.6);
            }
            else if(point == 5){ 
                Spot target_spot(372.1,-116.3);
            }
            else{
                ROS_WARN("arrival fail >> strange type : %d", point);
                return false;
            }    
        }
        
        dis = calculateDistance(current_x_, current_y_, target_spot.x, target_spot.y);        

        if (dis < 0.1){
            ROS_WARN("arrival success: %d", point);
            return true;
        }
        else{
            ROS_WARN("arrival fail: %d", point);
            return false;
        }
    }

    // 여기가 메인 state 함수임!!
    void state() {
        ros::Rate rate(20);  // 0.01 Hz
        while (ros::ok()) {

            int starting_point1 = 0;
            int arrivel_point1 = 1;
            bool is_indoor1 = true;
            // starting_point에서 arrival_point로 가라(실내/실외)
            // return: (int) arrival point (1번~5번) 지점
            // 실패하면 0 반환
            request_planning(starting_point1, arrivel_point1, is_indoor1);

            int starting_point2 = 0;
            int arrivel_point2 = 1;
            bool is_indoor2 = true;
            float detect_range1 = 10.0;
            // 돌아가면서 박을 수 있으면 박자 -> 제어권 넘김
            request_planning_with_collision(starting_point2, arrivel_point2, is_indoor2, detect_range1);            

            float detect_range2 = 10.0;
            // 사람에 부딪혀라 -> 안 쓸것같음
            request_collision_to_person(detect_range2);

            std::string teleport_point = "respawn_indoor";
            // 실내 respawn 지점으로 이동했는지 안 했는지 -> 이중체크 용도
            check_teleport_success(teleport_point);

            int arrival_point = 3;
            bool is_indoor = 3;
            check_arrival_success(arrival_point, is_indoor);

            // 완전히 정지한 상태로 픽업 및 배달
            // n번에서 픽업하면 '무조건' n번으로 배달
            int item_index1 = 1;
            // delivery
            delivery(item_index1);

            int item_index2 = 1;
            // pickup
            pickup(item_index2);

            // request planning 문자열 조합, state에서 publish, 확인용
            PublishWaypoints(waypoints_);
            
            rate.sleep();  // 지정된 주기로 대기
        }
    }

private:
    ros::NodeHandle nh_;

    actionlib::SimpleActionClient<morai_woowa::Planning_Tracking_ActAction> planning_tracking_ac_;
    actionlib::SimpleActionClient<morai_woowa::Person_Collision_ActAction> person_collision_ac_;

    ros::Subscriber current_pose_sub_;
    ros::Subscriber dilly_item_status_sub_;
    ros::Publisher waypoint_pub_; 

    ros::ServiceClient delivery_pickup_client_;
    ros::ServiceClient stop_tracking_client_;

    bool is_planning_action_finished_;
    bool is_reach_target_point_;
    float progress_percentage_;

    bool is_found_collision_target_;
    bool is_collision_action_finished_;
    bool is_success_collision_;

    path waypoints_;

    int closest_index_;
    double cross_track_error; 
    float current_x_;
    float current_y_;

    bool wpt_init_flag_;

    // 현재 적재중인 item list 및 개수
    std::vector<int> dilly_item_status_list_; 
    int dilly_item_status_cnt_;
    
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "state_node");

    StateNode StateNode;

    path waypoints;
    StateNode.loadWaypoints("/path/to/waypoints.csv", waypoints);
    StateNode.PublishWaypoints(waypoints);

    // std::thread thread(&StateNode::state, &StateNode);

    ros::spin();
    
    // thread.join();  

    return 0;
}