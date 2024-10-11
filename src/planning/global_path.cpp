#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include "Morai_Woowa/way_point.h"

typedef std::vector<Waypoint> path;

class GPathPub {
public:
    GPathPub() {
        // NodeHandle 초기화
        // Subscriber
        waypoint_sub_ = nh_.subscribe("/waypoints", 10, &GPathPub::waypointCallback, this);
        // Publisher
        gpath_pub_ = nh_.advertise<nav_msgs::Path>("/gpath", 10);  // 경로 퍼블리셔
    }

    void waypointCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        // 수신한 웨이포인트를 Waypoint 구조체로 변환하여 저장
        Waypoint wp;
        wp.x = msg->pose.position.x;
        wp.y = msg->pose.position.y;
        wp.heading = tf::getYaw(msg->pose.orientation);
        waypoints_.push_back(wp);
    }
    void publishPath() {
        // 웨이포인트가 없으면 퍼블리시하지 않음
        if (waypoints_.empty()) {
            ROS_WARN("No waypoints to publish.");
            return;
        }

        // nav_msgs::Path 메시지 생성
        nav_msgs::Path path_msg;
        path_msg.header.frame_id = "map"; // 사용할 프레임 설정
        path_msg.header.stamp = ros::Time::now();

        for (const auto& waypoint : waypoints_) {
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header = path_msg.header;

            // 웨이포인트 좌표를 PoseStamped 메시지에 삽입
            pose_stamped.pose.position.x = waypoint.x;
            pose_stamped.pose.position.y = waypoint.y;
            pose_stamped.pose.position.z = 0;  // Z축은 0으로 설정 (평면 상 경로)

            // heading 값을 쿼터니언으로 변환해 orientation에 설정
            pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(waypoint.heading);

            path_msg.poses.push_back(pose_stamped);
        }

        // Path 퍼블리시
        gpath_pub_.publish(path_msg);

        ROS_INFO("Published path with %zu waypoints.", waypoints_.size());

        // 퍼블리시 후 웨이포인트 리스트를 초기화할지 여부 결정
        // 필요에 따라 아래 주석을 제거하여 웨이포인트를 초기화할 수 있습니다.
        // waypoints_.clear();
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber waypoint_sub_;
    ros::Publisher gpath_pub_;

    path waypoints_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "GPathPub");

    GPathPub state_node;

    // 주기적으로 경로 퍼블리시
    ros::Rate rate(1);  // 1Hz
    while (ros::ok()) {
        ros::spinOnce();             // 콜백 함수 호출
        state_node.publishPath();    // 경로 퍼블리시
        rate.sleep();
    }

    return 0;
}
