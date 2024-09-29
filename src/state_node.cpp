#include <ros/ros.h>
#include <ros/package.h>  
#include <geometry_msgs/Pose2D.h>
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

class StateNode {
public:
    StateNode() : closest_index_(-1) {

        // path폴더안에 path파일 이름!!!!!
        std::string filename = "test_path.csv";
        //ros::param::get("~waypoint_file_name", filename); 

        std::string current_path = ros::package::getPath("Morai_Woowa"); // 패키지 경로를 가져옵니다
        waypoint_file_ = current_path + "/path/" + filename;

        current_pose_sub_ = nh_.subscribe("/current_pose", 10, &StateNode::currentPoseCallback, this);
        loadWaypoints();  // waypoint 파일에서 로드
    }

    void currentPoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg) {
        closest_index_ = findClosestWaypoint(msg->x, msg->y, closest_index_);
        if (closest_index_ != -1) {
            ROS_INFO("Current Position: (X: %.2f, Y: %.2f) is nearest to Waypoint Index: %d", msg->x, msg->y, closest_index_);
            ROS_INFO("cross_track_error: %lf", cross_track_error);
        } else {
            ROS_WARN("No waypoints available.");
        }
    }

    // 여기가 메인 state 함수임!!
    void state() {
        ros::Rate rate(100);  // 0.01 Hz
        while (ros::ok()) {

            ROS_INFO("closest_index: %d", closest_index_);

            rate.sleep();  // 지정된 주기로 대기
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber current_pose_sub_;
    std::vector<Waypoint> waypoints_;
    std::string waypoint_file_;
    int closest_index_ = 0;
    double cross_track_error = 10000000; 

    bool wpt_init_flag = false;

    void loadWaypoints() {
        
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

    int findClosestWaypoint(double x, double y, int previous_index) {

        // 처음엔 모든 wpt를 돌면서 위치 체크
        if(!wpt_init_flag){
            int closest_index = -1;
            double min_distance = std::numeric_limits<double>::max();

            for (int i = 0; i < waypoints_.size(); i++) {
                const auto& wp = waypoints_[i];
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
        int end_index = std::min(previous_index + 5, static_cast<int>(waypoints_.size()) - 1);

        int closest_index = -1;
        double min_distance = std::numeric_limits<double>::max();

        for (int i = start_index; i <= end_index; ++i) {

            const auto& wp = waypoints_[i];
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

    ros::spin();  // ROS 이벤트 루프 실행

    thread.join();  // 스레드가 종료될 때까지 대기

    ros::spin();

    return 0;
}
