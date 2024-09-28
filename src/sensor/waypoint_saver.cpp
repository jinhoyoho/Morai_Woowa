#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <vector>
#include <cmath>
#include <fstream>

struct Waypoint {
    double x;
    double y;
    double heading;
    int index;
};

class WaypointSaver {
public:
    WaypointSaver() : waypoint_index_(0) {
        current_pose_sub_ = nh_.subscribe("/current_pose", 10, &WaypointSaver::currentPoseCallback, this);
        ros::param::get("~waypoint_file", waypoint_file_);  // 매개변수에서 파일 경로를 읽음
    }

    ~WaypointSaver() {
        saveWaypoints();  // 노드 종료 시 waypoint 저장
    }

    void currentPoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg) {
        if (waypoints_.empty()) {
            // 첫 번째 waypoint 추가
            addWaypoint(msg->x, msg->y, msg->theta);
        } else {
            // 마지막 waypoint와의 거리 계산
            Waypoint& last_wp = waypoints_.back();
            double distance = calculateDistance(last_wp.x, last_wp.y, msg->x, msg->y);

            if (distance >= 0.3) {
                addWaypoint(msg->x, msg->y, msg->theta);
            }
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber current_pose_sub_;
    std::vector<Waypoint> waypoints_;
    int waypoint_index_;
    std::string waypoint_file_;

    double calculateDistance(double x1, double y1, double x2, double y2) {
        return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
    }

    void addWaypoint(double x, double y, double heading) {
        Waypoint waypoint;
        waypoint.x = x;
        waypoint.y = y;
        waypoint.heading = heading;
        waypoint.index = waypoint_index_++;
        
        waypoints_.push_back(waypoint);
        ROS_INFO("Waypoint saved: Index=%d, X=%.2f, Y=%.2f, Heading=%.2f", waypoint.index, waypoint.x, waypoint.y, waypoint.heading);
    }

    void saveWaypoints() {
        std::ofstream file(waypoint_file_);
        if (file.is_open()) {
            for (const auto& wp : waypoints_) {
                file << wp.index << "," << wp.x << "," << wp.y << "," << wp.heading << "\n";
            }
            file.close();
            ROS_INFO("Waypoints saved to %s", waypoint_file_.c_str());
        } else {
            ROS_ERROR("Unable to open file %s for writing", waypoint_file_.c_str());
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "waypoint_saver");

    WaypointSaver waypoint_saver;

    ros::spin();
    return 0;
}
