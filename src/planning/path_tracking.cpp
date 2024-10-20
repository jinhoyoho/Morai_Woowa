// path_tracking.cpp
#include "Morai_Woowa/path_tracking.h"
#include <tf/transform_datatypes.h> // Quaternion to yaw 변환
#include <morai_msgs/SkidSteer6wUGVCtrlCmd.h>
#include <cmath>
#include <limits>

// 생성자: Subscriber와 Publisher 초기화
PurePursuitController::PurePursuitController(ros::NodeHandle& nh) :
    wheel_base(3.9), lfd(2.0), steering(0), is_look_forward_point(false),
    current_linear_vel(0.0), current_angular_vel(0.0), vehicle_yaw(0.0),
    previous_time(ros::Time(0)), previous_heading(0.0), is_rotating_(false), target_heading_(0.0),
    current_waypoint_idx_(0), wpt_init_flag_(false), has_turned_left_(false),
    current_state_(FOLLOW_PATH)
{
    // Subscriber 초기화
    gpath_sub_ = nh.subscribe("/lpath", 10, &PurePursuitController::publishPath, this);  // Waypoint 구독
    current_pose_sub_ = nh.subscribe("/current_pose", 10, &PurePursuitController::getRobotStatus, this);
    odom_sub_ = nh.subscribe("/odom", 10, &PurePursuitController::odomCallback, this);
    // Publisher 초기화
    ctrl_cmd_pub_ = nh.advertise<morai_msgs::SkidSteer6wUGVCtrlCmd>("/6wheel_skid_ctrl_cmd", 10);
}

// Odometry 콜백 함수: 선속도와 각속도 업데이트
void PurePursuitController::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    current_linear_vel = msg->twist.twist.linear.x;       // 선속도 (m/s)
    current_angular_vel = msg->twist.twist.angular.z;     // 각속도 (rad/s)
}

// Pose 콜백 함수: 현재 위치와 헤딩 업데이트
void PurePursuitController::getRobotStatus(const geometry_msgs::PoseStamped::ConstPtr& pose_msg) {
    // 현재 위치 업데이트
    current_position.x = pose_msg->pose.position.x;
    current_position.y = pose_msg->pose.position.y;

    // Quaternion을 Yaw로 변환
    tf::Quaternion q(
        pose_msg->pose.orientation.x,
        pose_msg->pose.orientation.y,
        pose_msg->pose.orientation.z,
        pose_msg->pose.orientation.w
    );
    tf::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, vehicle_yaw);  // Yaw 값 (라디안)

    // 현재 위치 출력
    ROS_INFO("Current Position -> x: %.2f, y: %.2f, yaw: %.2f", current_position.x, current_position.y, vehicle_yaw);

    // // 현재 시간 업데이트
    // ros::Time current_time = ros::Time::now();

    // if (!previous_time.isZero()) {
    //     // 시간 차이 계산
    //     double delta_time = (current_time - previous_time).toSec();
    //     // 헤딩 차이 계산
    //     double delta_heading = vehicle_yaw - previous_heading;

    //     // 헤딩 차이 보정 ([-pi, pi] 범위로)
    //     if (delta_heading > M_PI) {
    //         delta_heading -= 2 * M_PI;
    //     } else if (delta_heading < -M_PI) {
    //         delta_heading += 2 * M_PI;
    //     }

    //     // 각속도 계산
    //     double angular_velocity = delta_heading / delta_time;

    //     // 각속도 출력 (디버깅용)
    //     // ROS_INFO("Angular Velocity: %.2f rad/s", angular_velocity);
    // }

    // // 상태 업데이트
    // previous_heading = vehicle_yaw;
    // previous_time = current_time;

    // Look-ahead 거리 동적 조정
    lfd = 2.0 + 0.5 * current_linear_vel;
    // ROS_INFO("Dynamic LFD: %.2f, Current Velocity: %.2f, Angular Velocity: %.2f", lfd, current_linear_vel, current_angular_vel);
}

// Path 콜백 함수: 웨이포인트 업데이트
void PurePursuitController::publishPath(const nav_msgs::Path::ConstPtr& msg) {
    waypoint_path.clear();

    for (const auto& pose_stamped : msg->poses) {
        Waypoint wp;
        wp.x = pose_stamped.pose.position.x;
        wp.y = pose_stamped.pose.position.y;
        wp.heading = tf::getYaw(pose_stamped.pose.orientation);
        waypoint_path.push_back(wp);
    }

    // ROS_INFO("Received %zu waypoints.", waypoint_path.size());
}

// 스티어링 각도 계산 함수
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

// 곡률 계산 함수
double PurePursuitController::calculateCurvature() {
    if (waypoint_path.size() < 3) {
        return 0.0;
    }

    // 현재 위치에 가장 가까운 웨이포인트 인덱스 찾기
    size_t idx = 0;
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

    // 3, 4, 5번째 웨이포인트가 로봇 뒤에 있는지 확인
    if (!has_turned_left_ && (isWaypointBehind(x3, y3) || isWaypointBehind(x2, y2) || isWaypointBehind(x1, y1))) {
        current_state_ = TURN_LEFT_90;
        ROS_INFO("Detected waypoints behind the robot. Switching to TURN_LEFT_90 state.");
    }

    // 곡률 계산 (kappa)
    double kappa = fabs((x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1)) /
                   (pow((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1), 1.5) + 1e-6);
    return kappa;
}

// 웨이포인트가 로봇 뒤에 있는지 확인하는 함수
bool PurePursuitController::isWaypointBehind(double wx, double wy) {
    double dx = wx - current_position.x;
    double dy = wy - current_position.y;

    // 좌표 변환
    double transformed_x = cos(vehicle_yaw) * dx + sin(vehicle_yaw) * dy;

    // 변환된 x가 0보다 작으면 로봇 뒤에 있는 것으로 간주
    return transformed_x < 0;
}



// 상태별 함수: FOLLOW_PATH
void PurePursuitController::followPath(ros::Rate& rate) {
    // 스티어링 각도와 곡률 계산
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

    // 스티어링 각도를 각속도로 변환 (degrees -> radians)
    double target_angular_velocity = angle * M_PI / 180.0;

    // 각속도를 최대값으로 제한
    if (target_angular_velocity > 0.83) {
        target_angular_velocity = 0.83;
    } else if (target_angular_velocity < -0.83) {
        target_angular_velocity = -0.83;
    }

    ROS_INFO("Speed: %.2f, Angular Velocity: %.2f", speed, target_angular_velocity);

    // 목표 지점에 가까워지면 정지
    if (!waypoint_path.empty()) {
        Waypoint last_point = waypoint_path.back();
        double dist_to_goal = hypot(last_point.x - current_position.x, last_point.y - current_position.y);

        ROS_INFO("Distance to goal: %.2f", dist_to_goal);
        if (dist_to_goal < 0.4) {  // 0.4m 임계값
            current_state_ = AT_GOAL;
            Brake();
            ROS_INFO("Reached final point and stopped.");
            return;
        }
    }

    // 제어 명령 발행
    morai_msgs::SkidSteer6wUGVCtrlCmd ctrl_cmd;
    ctrl_cmd.cmd_type = 3;
    ctrl_cmd.Target_linear_velocity = speed;
    ctrl_cmd.Target_angular_velocity = target_angular_velocity;
    ctrl_cmd_pub_.publish(ctrl_cmd);
    ROS_INFO("Publishing Command: Linear Velocity = %.2f, Angular Velocity = %.2f, cmd_type = %d", speed, target_angular_velocity, ctrl_cmd.cmd_type);
}

void PurePursuitController::turnLeft(ros::Rate& rate) {
    if (!is_rotating_) {
        target_heading_ = vehicle_yaw + M_PI / 2;  // 90도 회전

        // 2π를 초과하거나 미만일 경우 보정
        if (target_heading_ > M_PI) {
            target_heading_ -= 2 * M_PI;
        } else if (target_heading_ <= -M_PI) {
            target_heading_ += 2 * M_PI;
        }

        is_rotating_ = true;
        ROS_INFO("Initiating turnLeft within turnLeft state. Target heading: %.2f radians", target_heading_);
    }

    double angle_diff = target_heading_ - vehicle_yaw;
    // 각도 차이 보정 ([-pi, pi] 범위)
    if (angle_diff > M_PI) {
        angle_diff -= 2 * M_PI;
    } else if (angle_diff < -M_PI) {
        angle_diff += 2 * M_PI;
    }

    if (fabs(angle_diff) < 0.1) {  //
        is_rotating_ = false;
        has_turned_left_ = true;
        current_state_ = FOLLOW_PATH;
        ROS_INFO("Completed turning. Switching back to FOLLOW_PATH state.");
        Brake();
    } else {
        // 계속 회전 명령 보내기
        morai_msgs::SkidSteer6wUGVCtrlCmd ctrl_cmd;
        ctrl_cmd.cmd_type = 3;
        ctrl_cmd.Target_linear_velocity = 0;
        ctrl_cmd.Target_angular_velocity = (angle_diff > 0) ? -0.83 : 0.83;  // 회전 방향에 따라 설정
        ctrl_cmd_pub_.publish(ctrl_cmd);
        ROS_INFO("Turning... Current heading: %.2f, Target heading: %.2f, Angle diff: %.2f", vehicle_yaw, target_heading_, angle_diff);
        rate.sleep();
    }
}
// 제어 루프 함수
void PurePursuitController::controlLoop() {
    ros::Rate rate(10);
    bool at_goal = false;

    ROS_INFO("Starting control loop.");

    // 웨이포인트가 수신될 때까지 대기
    while (waypoint_path.empty() && ros::ok()) {
        ROS_WARN("Waiting for waypoints to be received...");
        ros::spinOnce();
        rate.sleep();
    }

    while (ros::ok()) {
        ros::spinOnce();  // 콜백 함수 호출

        if (waypoint_path.empty()) {
            ROS_WARN("No waypoints available.");
            rate.sleep();
            continue;
        }

        switch(current_state_) {
            case FOLLOW_PATH:
                followPath(rate);
                break;
            
            case TURN_LEFT_90:
                turnLeft(rate);
                break;
            
            case AT_GOAL:
                Brake();
                ROS_INFO("At goal. Stopping.");
                break;
        }

        rate.sleep();
    }
}

// 브레이크 함수: 정지 명령 발행
void PurePursuitController::Brake() {
    morai_msgs::SkidSteer6wUGVCtrlCmd ctrl_cmd;
    ctrl_cmd.cmd_type = 3;
    ctrl_cmd.Target_linear_velocity = 0;
    ctrl_cmd.Target_angular_velocity = 0;
    ctrl_cmd_pub_.publish(ctrl_cmd);
    ROS_INFO("Brake Activated!");
}
// findClosestWaypoint 함수 구현
int PurePursuitController::findClosestWaypoint(double x, double y, int previous_index, const std::vector<Waypoint>& waypoints) {
    double min_dist = std::numeric_limits<double>::max();
    int closest_idx = previous_index;

    for (size_t i = previous_index; i < waypoints.size(); ++i) {
        double dist = hypot(waypoints[i].x - x, waypoints[i].y - y);
        if (dist < min_dist) {
            min_dist = dist;
            closest_idx = i;
        }
    }

    return closest_idx;
}
// 메인 함수
int main(int argc, char** argv) {
    ros::init(argc, argv, "path_tracking");
    ros::NodeHandle nh;
    PurePursuitController controller(nh);
    controller.controlLoop();
    return 0;
}