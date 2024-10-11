#include "Morai_Woowa/path_tracking.h"
#include <tf/transform_datatypes.h> // 쿼터니언 변환을 위해 사용
#include <morai_msgs/SkidSteer6wUGVCtrlCmd.h>

PurePursuitController::PurePursuitController(ros::NodeHandle& nh) :
    wheel_base(3.9), lfd(2.0), steering(0), is_look_forward_point(false),
    current_linear_vel(0.0), current_angular_vel(0.0), vehicle_yaw(0.0),
    previous_time(ros::Time(0)), previous_heading(0.0)
{
    // Subscriber
    gpath_sub_ = nh.subscribe("/gpath", 10, &PurePursuitController::publishPath, this);  // Waypoint 구독
    current_pose_sub_ = nh.subscribe("/current_pose", 10, &PurePursuitController::getRobotStatus, this);
    odom_sub_ = nh.subscribe("/odom", 10, &PurePursuitController::odomCallback, this);
    // Publisher
    ctrl_cmd_pub_ = nh.advertise<morai_msgs::SkidSteer6wUGVCtrlCmd>("/6wheel_skid_ctrl_cmd", 10);
}

void PurePursuitController::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    current_linear_vel = msg->twist.twist.linear.x;  // 선속도를 받아옴 (m/s 단위)
    current_angular_vel = msg->twist.twist.angular.z; // 각속도도 받아옴
}

void PurePursuitController::getRobotStatus(const geometry_msgs::PoseStamped::ConstPtr& pose_msg) {
    // 현재 위치를 current_pose 메시지에서 가져옴
    current_position.x = pose_msg->pose.position.x;
    current_position.y = pose_msg->pose.position.y;
    // Quaternion to yaw (오리엔테이션 쿼터니언을 Yaw 값으로 변환)
    tf::Quaternion q(
        pose_msg->pose.orientation.x,
        pose_msg->pose.orientation.y,
        pose_msg->pose.orientation.z,
        pose_msg->pose.orientation.w
    );
    tf::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, vehicle_yaw);  // Yaw 값을 얻음 (라디안 단위)
    // 현재 시간 얻기
    ros::Time current_time = ros::Time::now();

    if (!previous_time.isZero()) {
        // 시간 변화량 계산
        double delta_time = (current_time - previous_time).toSec();
        // 각도 변화량 계산
        double delta_heading = vehicle_yaw - previous_heading;

        // 각도 차이가 너무 크다면 (±π 이상의 경우) 값 보정 (2π 주기 보정)
        if (delta_heading > M_PI) {
            delta_heading -= 2 * M_PI;
        } else if (delta_heading < -M_PI) {
            delta_heading += 2 * M_PI;
        }

        // 각속도 계산
        double angular_velocity = delta_heading / delta_time;

        // 계산된 각속도를 출력하거나 저장
        ROS_INFO("Angular Velocity: %.2f rad/s", angular_velocity);
    }
    // 상태 업데이트 (다음 번 계산을 위해)
    previous_heading = vehicle_yaw;
    previous_time = current_time;

    // lfd를 동적으로 조정 (기본 lfd 2.0 + 속도 계수 0.5 적용)
    lfd = 2.0 + 0.5 * current_linear_vel;
    ROS_INFO("Dynamic LFD: %.2f, Current Velocity: %.2f, Angular Velocity: %.2f", lfd, current_linear_vel, current_angular_vel);
}

void PurePursuitController::publishPath(const nav_msgs::Path::ConstPtr& msg) {
    waypoint_path.clear();

    for (const auto& pose_stamped : msg->poses) {
        Waypoint wp;
        wp.x = pose_stamped.pose.position.x;
        wp.y = pose_stamped.pose.position.y;
        wp.heading = tf::getYaw(pose_stamped.pose.orientation);
        waypoint_path.push_back(wp);
    }

    ROS_INFO("Received %zu waypoints.", waypoint_path.size());
}

double PurePursuitController::steering_angle() {
    is_look_forward_point = false;
    geometry_msgs::Point rotated_point;

    for (const auto& wp : waypoint_path) {
        // 좌표 변환: 로봇 기준으로 경로점 위치 변환
        double dx = wp.x - current_position.x;
        double dy = wp.y - current_position.y;

        rotated_point.x = cos(vehicle_yaw) * dx + sin(vehicle_yaw) * dy;
        rotated_point.y = sin(vehicle_yaw) * dx - cos(vehicle_yaw) * dy;

        if (rotated_point.x > 0) {
            double dis = sqrt(pow(rotated_point.x, 2) + pow(rotated_point.y, 2));
            if (dis >= lfd) {
                forward_point = wp;
                is_look_forward_point = true;
                break;
            }
        }
    }
    // 스티어링 각도 계산
    if (is_look_forward_point) {
        double theta = atan2(rotated_point.y, rotated_point.x);
        steering = atan2(2 * wheel_base * sin(theta), lfd) * 180.0 / M_PI;
    } else {
        steering = 0;
    }
    return steering;
}

double PurePursuitController::calculateCurvature() {
    if (waypoint_path.size() < 3) {
        return 0.0;
    }

    // 현재 위치에서 가까운 세 개의 웨이포인트 사용
    size_t idx = 0; // 현재 위치에 가장 가까운 웨이포인트 인덱스
    double min_dist = std::numeric_limits<double>::max();

    for (size_t i = 0; i < waypoint_path.size(); ++i) {
        double dist = hypot(waypoint_path[i].x - current_position.x, waypoint_path[i].y - current_position.y);
        if (dist < min_dist) {
            min_dist = dist;
            idx = i;
        }
    }

    if (idx + 2 >= waypoint_path.size()) {
        return 0.0;
    }

    // 세 개의 웨이포인트 좌표
    double x1 = waypoint_path[idx].x;
    double y1 = waypoint_path[idx].y;
    double x2 = waypoint_path[idx + 1].x;
    double y2 = waypoint_path[idx + 1].y;
    double x3 = waypoint_path[idx + 2].x;
    double y3 = waypoint_path[idx + 2].y;

    // 곡률 계산
    double kappa = fabs((x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1)) /
                   (pow((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1), 1.5) + 1e-6);

    return kappa;
}

void PurePursuitController::controlLoop() {
    ros::Rate rate(10);
    while (ros::ok()) {
        ros::spinOnce();  // 콜백 함수 호출
        double angle = steering_angle();
        double curvature = calculateCurvature();

        // 곡률에 따른 속도 조절
        double max_speed = 3.0; // 최대 속도 (m/s)
        double min_speed = 1.0; // 최소 속도 (m/s)
        double speed;

        if (curvature < 0.1) {
            speed = max_speed;
        } else {
            speed = max_speed / (1 + 10 * curvature);
            if (speed < min_speed) {
                speed = min_speed;
            }
        }

        // 각속도로 변환 (degrees -> radians)
        double target_angular_velocity = angle * M_PI / 180.0;

        // 각속도를 최대값으로 제한
        if (target_angular_velocity > 0.83) {
            target_angular_velocity = 0.83;
        } else if (target_angular_velocity < -0.83) {
            target_angular_velocity = -0.83;
        }

        morai_msgs::SkidSteer6wUGVCtrlCmd ctrl_cmd;
        ctrl_cmd.cmd_type = 3;
        ctrl_cmd.Target_linear_velocity = speed;
        ctrl_cmd.Target_angular_velocity = target_angular_velocity;
        ctrl_cmd_pub_.publish(ctrl_cmd);
        rate.sleep();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_tracking");
    ros::NodeHandle nh;
    PurePursuitController controller(nh);
    controller.controlLoop();
    return 0;
}