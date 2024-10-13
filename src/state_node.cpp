#include <ros/ros.h>
#include <ros/package.h>  
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include "morai_msgs/WoowaDillyEventCmdSrv.h"
#include <tf/transform_datatypes.h>

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
    planning_tracking_ac_(nh_, "planning_tracking_action", true),
    person_collision_ac_(nh_, "person_collision_action", true)
    {
        current_pose_sub_ = nh_.subscribe("/current_pose", 10, &StateNode::currentPoseCallback, this);
        waypoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/waypoints", 10);  // PoseStamped 퍼블리셔
        waypoint_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/globalpath", 10, this);

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

    bool pickup(int item_index){

        morai_msgs::WoowaDillyEventCmdSrv pick_srv;
        pick_srv.request.request.deliveryItemIndex = item_index;
        pick_srv.request.request.isPickup = true;

        delivery_pickup_client_.call(pick_srv);

        return pick_srv.response.response.result;
    }

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

    int request_planning_point(int a, int b, bool indoor){

        std::ostringstream oss1;
        std::ostringstream oss2;
        std::string filename;
        std::string out_or_in;
        morai_woowa::Planning_Tracking_ActGoal planning_goal;

        if (indoor) 
            out_or_in = "indoor";
        else
            out_or_in = "outdoor";

        oss1 << out_or_in << "_" << a << "_" << b << ".csv";
        filename = oss1.str();

        std::ifstream file(filename);
        if (file.is_open()) {
            planning_goal.path = filename;  
            planning_goal.reverse = false;  
        }
        else {
            oss2 << out_or_in << "_" << b << "_" << a << ".csv";
            filename = oss2.str();
            std::ifstream file(filename);

            if (file.is_open()) {
                planning_goal.path = filename;
                planning_goal.reverse = true;  
            }
            else {
                ROS_WARN("Unable to open waypoint file: %s", filename.c_str());
                return false;
            }
        }
        loadWaypoints(filename, waypoints_);

        planning_tracking_ac_.sendGoal(planning_goal,
                    boost::bind(&StateNode::PlanningActionDoneCb, this, boost::placeholders::_1, boost::placeholders::_2),
                    boost::bind(&StateNode::PlanningActionActiveCb, this),
                    boost::bind(&StateNode::PlanningActionFeedbackCb, this, boost::placeholders::_1));

        is_planning_action_finished_ = false;
        is_reach_target_point_ = false;
        progress_percentage_ = 0;

        ros::Rate loop_rate(0.5);

        while (!is_planning_action_finished_ && ros::ok()){
            loop_rate.sleep();
            ros::spinOnce();
        }

        if(is_reach_target_point_)
            return b; 
        else
            return 0;
    }

    int request_planning_with_collision(int a, int b, bool indoor, float range){

        std::ostringstream oss1;
        std::ostringstream oss2;
        std::string filename;
        std::string out_or_in;
        morai_woowa::Planning_Tracking_ActGoal planning_goal;

        if (indoor) 
            out_or_in = "indoor";
        else
            out_or_in = "outdoor";

        oss1 << out_or_in << "_" << a << "_" << b << ".csv";
        filename = oss1.str();

        std::ifstream file(filename);
        if (file.is_open()) {
            planning_goal.path = filename;  
            planning_goal.reverse = false;  
        }
        else {
            oss2 << out_or_in << "_" << b << "_" << a << ".csv";
            filename = oss2.str();
            std::ifstream file(filename);

            if (file.is_open()) {
                planning_goal.path = filename;
                planning_goal.reverse = true;  
            }
            else {
                ROS_WARN("Unable to open waypoint file: %s", filename.c_str());
                return false;
            }
        }
        loadWaypoints(filename, waypoints_);

        //
        planning_tracking_ac_.sendGoal(planning_goal,
                    boost::bind(&StateNode::PlanningActionDoneCb, this, boost::placeholders::_1, boost::placeholders::_2),
                    boost::bind(&StateNode::PlanningActionActiveCb, this),
                    boost::bind(&StateNode::PlanningActionFeedbackCb, this, boost::placeholders::_1));

        is_planning_action_finished_ = false;
        is_reach_target_point_ = false;
        progress_percentage_ = 0;

        //
        morai_woowa::Person_Collision_ActGoal collision_goal;
        collision_goal.range = range; 

        person_collision_ac_.sendGoal(collision_goal,
                    boost::bind(&StateNode::PersonCollisionActionDoneCb, this, boost::placeholders::_1, boost::placeholders::_2),
                    boost::bind(&StateNode::PersonCollisionActionActiveCb, this),
                    boost::bind(&StateNode::PersonCollisionActionFeedbackCb, this, boost::placeholders::_1));


        is_collision_action_finished_ = false;
        is_success_collision_ = false;
        is_found_collision_target_ = false;

        bool tracking_stop_flag = false;

        ros::Rate loop_rate(0.5);

        while (!is_planning_action_finished_ && ros::ok()){
            
            if(is_found_collision_target_ && progress_percentage_ < 0.25 && !tracking_stop_flag){
                
                tracking_stop_flag = true; 

                //충돌타겟 발견시 일단 트래킹 프로세스는 멈춰
                morai_woowa::StopTrackingSrv stop_srv;
                stop_srv.request.stop = true;
                stop_tracking_client_.call(stop_srv);

                //충돌에 집중
                while (!is_collision_action_finished_ && ros::ok()){
                    loop_rate.sleep();
                    ros::spinOnce();
                }
                
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
            return b; 
        else
            return a;
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
        
        if (type == "teloport_indoor"){
            Spot teloport_spot_indoor(3.00, -42.00);
            dis = calculateDistance(current_x_, current_y_, teloport_spot_indoor.x, teloport_spot_indoor.y);
        }
        else if (type == "teloport_outdoor"){
            Spot teloport_spot_outdoor(430.00, -140.00);
            dis = calculateDistance(current_x_, current_y_, teloport_spot_outdoor.x, teloport_spot_outdoor.y);        
        }
        else if (type == "respawn_indoor"){
            Spot respawn_spot_indoor(430.00, -140.00);
            dis = calculateDistance(current_x_, current_y_, respawn_spot_indoor.x, respawn_spot_indoor.y);        
        }
        else if (type == "respawn_outdoor"){
            Spot respawn_spot_outdoor(430.00, -140.00);
            dis = calculateDistance(current_x_, current_y_, respawn_spot_outdoor.x, respawn_spot_outdoor.y);        
        }
        else{
            ROS_WARN("teleport fail >> strange type : %s", type.c_str());
            return false;
        }    

        if (dis < 1){
            ROS_WARN("teleport success: %s", type.c_str());
            return true;
        }
        else{
            ROS_WARN("teleport fail: %s", type.c_str());
            return false;
        }
    }

    // 여기가 메인 state 함수임!!
    void state() {
        ros::Rate rate(20);  // 0.01 Hz
        while (ros::ok()) {

            request_planning_point(0,1,true);

            request_collision_to_person(10);

            check_teleport_success("respawn_indoor");

            delivery(1);

            pickup(1);

            PublishWaypoints(waypoints_);
            
            rate.sleep();  // 지정된 주기로 대기
        }
    }

private:
    ros::NodeHandle nh_;

    actionlib::SimpleActionClient<morai_woowa::Planning_Tracking_ActAction> planning_tracking_ac_;
    actionlib::SimpleActionClient<morai_woowa::Person_Collision_ActAction> person_collision_ac_;

    ros::Subscriber current_pose_sub_;
    ros::Publisher waypoint_pub_; 
    ros::Publisher waypoint_marker_pub_;

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