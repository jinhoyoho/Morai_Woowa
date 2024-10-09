#include <ros/ros.h>
#include <ros/package.h>  
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

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

class StateNode {
public:
    StateNode() : closest_index_(-1) {

        // path폴더안에 path파일 이름!!!!!
        // std::string filename = "test_path.csv";
        // std::string current_path = ros::package::getPath("Morai_Woowa"); // 패키지 경로를 가져옵니다
        // waypoint_file_ = current_path + "/path/" + filename;

        nh_.param<std::string>("/state_node/waypoint_file1", waypoint_file_1_, "waypoints.csv");
        nh_.param<std::string>("/state_node/waypoint_file2", waypoint_file_2_, "waypoints.csv");
        nh_.param<std::string>("/state_node/waypoint_file3", waypoint_file_3_, "waypoints.csv");


        current_pose_sub_ = nh_.subscribe("/current_pose", 10, &StateNode::currentPoseCallback, this);
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

    void pub_global_path(){
        publishWaypoints(waypoints_1_);
        publishWaypoints(waypoints_2_);
        publishWaypoints(waypoints_3_);
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

    // 여기가 메인 state 함수임!!
    void state() {
        ros::Rate rate(100);  // 0.01 Hz
        while (ros::ok()) {

            ////// 여기다 실행할 함수 //////

            ROS_INFO("closest_index: %d", closest_index_);


            pub_global_path();
            rate.sleep();  // 지정된 주기로 대기
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber current_pose_sub_;
    ros::Publisher waypoint_marker_pub_;

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