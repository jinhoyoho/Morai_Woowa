#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <vector>
#include <fstream>
#include <sstream>
#include <cmath>
#include <limits>

struct Waypoint {
    double x;
    double y;
    double heading;
    int index;
};

class StateNode {
public:
    StateNode() : closest_index_(-1) {
        current_pose_sub_ = nh_.subscribe("/current_pose", 10, &StateNode::currentPoseCallback, this);
        ros::param::get("~waypoint_file", waypoint_file_);  // 매개변수에서 파일 경로를 읽음
        loadWaypoints();  // waypoint 파일에서 로드
    }

    void currentPoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg) {
        closest_index_ = findClosestWaypoint(msg->x, msg->y, closest_index_);
        if (closest_index_ >= 0) {
            ROS_INFO("Current Position: (X: %.2f, Y: %.2f) is nearest to Waypoint Index: %d", msg->x, msg->y, closest_index_);
        } else {
            ROS_WARN("No waypoints available.");
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber current_pose_sub_;
    std::vector<Waypoint> waypoints_;
    std::string waypoint_file_;
    int closest_index_;

    void loadWaypoints() {
        std::ifstream file(waypoint_file_);
        if (file.is_open()) {
            Waypoint wp;
            while (file >> wp.index) {
                char comma;  // CSV를 읽기 위해 필요한 변수
                file >> comma; // Skip comma
                file >> wp.x;
                file >> comma; // Skip comma
                file >> wp.y;
                file >> comma; // Skip comma
                file >> wp.heading;
                waypoints_.push_back(wp);
            }
            file.close();
            ROS_INFO("Loaded waypoints from %s", waypoint_file_.c_str());
        } else {
            ROS_WARN("Unable to open waypoint file: %s", waypoint_file_.c_str());
        }
    }

    int findClosestWaypoint(double x, double y, int previous_index) {
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

    StateNode state_node;

    ros::spin();
    return 0;
}
