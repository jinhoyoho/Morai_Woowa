#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdexcept>

int main(int argc, char** argv) {
    ros::init(argc, argv, "csv_path_publisher");
    ros::NodeHandle nh;
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("path_topic", 10);

    // Create a Path message object
    nav_msgs::Path path;
    path.header.frame_id = "map";  // Set the frame ID to an appropriate value

    // Open the CSV file

    // 여기서 파일 이름만 바꿔
    std::ifstream file("/home/leesh/catkin_ws/src/Morai_Woowa/path/test_path.csv");
    if (!file.is_open()) {
        ROS_ERROR("Failed to open file.");
        return -1;
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
        ROS_INFO("Read values: x_str=%s, y_str=%s", x_str.c_str(), y_str.c_str());

        // Check if x and y values are not empty
        if (x_str.empty() || y_str.empty()) {
            ROS_WARN("Empty x or y value, skipping line");
            continue;  // Skip this line if x or y is empty
        }

        try {
            float x = std::stof(x_str);
            float y = std::stof(y_str);

            // Create a PoseStamped object for each point
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "map";  // Set the frame ID for each pose
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            pose.pose.position.z = 0.0;  // z-coordinate is 0 for 2D points

            // Orientation is set to a default value (if needed, heading could be used for yaw)
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;

            // Add the pose to the path
            path.poses.push_back(pose);
        } catch (const std::invalid_argument& e) {
            ROS_WARN("Invalid data encountered: %s", e.what());
            continue;  // Skip this line if there's an invalid value
        } catch (const std::out_of_range& e) {
            ROS_WARN("Out of range error: %s", e.what());
            continue;  // Skip this line if values are out of range
        }
    }

    file.close();

    ros::Rate loop_rate(1);
    while (ros::ok()) {
        path.header.stamp = ros::Time::now();
        path_pub.publish(path);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
