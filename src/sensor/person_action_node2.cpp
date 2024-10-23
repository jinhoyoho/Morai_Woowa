#include "Morai_Woowa/person_action_node2.h"

person_action_node::person_action_node(ros::NodeHandle& nh):PCAserver_(nh, "person_collision_action2", boost::bind(&person_action_node::execute, this, _1), false)
{
    PCAserver_.start(); // 액션 서버 시작
    current_pose_sub_ = nh.subscribe("/current_pose", 10, &person_action_node::currentPoseCallback, this);

    ctrl_cmd_pub_ = nh.advertise<morai_msgs::SkidSteer6wUGVCtrlCmd>("/collision_ctrl", 10);
    path_pub_ = nh.advertise<nav_msgs::Path>("person_collision_path", 10);
    
    current_x_ = 0;
    current_y_ = 0;
}

bool person_action_node::check_collision_success(){

    float dis = 10000;
    
    Spot respawn_spot_indoor(8.33, -38.92);// 실내 리스폰 지점
    dis = calculateDistance(current_x_, current_y_, respawn_spot_indoor.x, respawn_spot_indoor.y);        

    if (dis < 1){
        ROS_WARN("indoor teleport success");
        return true;
    }

    Spot respawn_spot_outdoor(430.00, -140.00);// 야외 리스폰 지점
    dis = calculateDistance(current_x_, current_y_, respawn_spot_outdoor.x, respawn_spot_outdoor.y);        

    if (dis < 1){
        ROS_WARN("outdoor teleport success");
        return true;
    }

    else{
        //ROS_WARN("teleport fail");
        return false;
    }
}

// 반원 경로 생성 함수
std::vector<Spot> person_action_node::generateSemiCircularPath(const Spot& start, const Spot& goal, int num_points) {
//     std::vector<Spot> path;

//     // 시작점과 목표점 사이의 중간점 계산
//     Spot mid;
//     mid.x = (start.x + goal.x) / 2.0;
//     mid.y = (start.y + goal.y) / 2.0;

//     // 시작점에서 목표점까지의 방향 계산
//     double direction = atan2(goal.y - start.y, goal.x - start.x);
//     std::cout << "Direction to goal: " << direction << std::endl;

//     // 반지름 계산 (중간점에서 시작점/목표점까지의 거리)
//     double radius = calculateDistance(mid, start);

//     // 반원을 그리기 위한 각도 계산 (반원을 적당히 나누기 위한 각도)
//     double theta_step = M_PI / num_points;  // 반원을 num_points로 나눔

//     // 로봇의 현재 yaw와 목표까지의 방향 차이
//     double angle_diff = direction - current_yaw_;

//     // 방향 차이를 통해 반원이 어느 방향으로 그려질지 결정
//     if (angle_diff > 0) {  // 목표 방향이 로봇의 오른쪽에 있음 (반원을 오른쪽으로 그림)
//         for (int i = 0; i <= num_points; ++i) {
//             Spot p;
//             double theta = i * theta_step;  // 각도를 점차 증가시키면서 반원을 그림

//             // 좌표계의 반원 방정식을 이용한 계산 (오른쪽 반원)
//             p.x = mid.x + radius * std::cos(direction - theta);
//             p.y = mid.y + radius * std::sin(direction - theta);

//             path.push_back(p);
//         }
//     } else {  // 목표 방향이 로봇의 왼쪽에 있음 (반원을 왼쪽으로 그림)
//         for (int i = 0; i <= num_points; ++i) {
//             Spot p;
//             double theta = i * theta_step;  // 각도를 점차 증가시키면서 반원을 그림

//             // 좌표계의 반원 방정식을 이용한 계산 (왼쪽 반원)
//             p.x = mid.x + radius * std::cos(direction + theta);
//             p.y = mid.y + radius * std::sin(direction + theta);

//             path.push_back(p);
//         }
//     }

//     return path;
// }

    std::vector<Spot> path;

    // 시작점에서 목표점까지의 중간점 계산
    Spot mid;
    mid.x = (start.x + goal.x) / 2.0;
    mid.y = (start.y + goal.y) / 2.0;

    // 시작점과 목표점 사이의 방향 계산
    double direction = atan2(goal.y - start.y, goal.x - start.x);
    std::cout << "Direction to goal: " << direction << std::endl;

    // 로봇과 목표 지점 사이의 거리 계산
    double distance = calculateDistance(start, goal);

    // 삼각형 꼭짓점 위치 계산 (중간점에서 수직으로 일정 거리만큼 이동한 지점)
    double height = distance * 2.0;  // 삼각형의 높이 (경유 지점의 높이)
    apex;  // 삼각형 꼭짓점(경유 지점)
    apex.x = mid.x + height * std::cos(direction + M_PI_2);  // 수직 방향으로 이동
    apex.y = mid.y + height * std::sin(direction + M_PI_2);

    // 시작점 -> 경유점(삼각형 꼭짓점) 경로 생성
    for (int i = 0; i <= num_points / 2; ++i) {
        Spot p;
        double t = static_cast<double>(i) / (num_points / 2);
        p.x = start.x + t * (apex.x - start.x);
        p.y = start.y + t * (apex.y - start.y);
        path.push_back(p);
    }

    // 경유점 -> 목표점 경로 생성
    for (int i = 0; i <= num_points / 2; ++i) {
        Spot p;
        double t = static_cast<double>(i) / (num_points / 2);
        p.x = apex.x + t * (goal.x - apex.x);
        p.y = apex.y + t * (goal.y - apex.y);
        path.push_back(p);
    }

    return path;
}


Spot person_action_node::load_spot(int spot, bool is_indoor){
    
    Spot person_spot;
    
    if(is_indoor){
        if(spot == 4)
            person_spot = {-77, 7};
        else if(spot == 5)
            person_spot = {30.8, -42.5};

    }

    return person_spot;
}

void person_action_node::execute(const morai_woowa::Person_Collision_Act2GoalConstPtr& goal)
{
    morai_woowa::Person_Collision_Act2Feedback feedback;
    morai_woowa::Person_Collision_Act2Result result;

    //ROS_INFO("Execute action: Person Collision Action!");

    Spot goal_person = load_spot(goal->spot, goal->is_indoor);

    std::cout << goal_person.x << " " << goal_person.y << std::endl;

    // 현재 위치와 목표 위치 설정
    Spot current_pos = {current_x_, current_y_};


    ros::Time t = ros::Time::now();

    while(ros::Time::now()-t < ros::Duration(1)){
        morai_msgs::SkidSteer6wUGVCtrlCmd ctrl_cmd;
        ctrl_cmd.cmd_type = 3;
        ctrl_cmd.Target_linear_velocity = -1.0;
        ctrl_cmd.Target_angular_velocity = 0;//target_angular_velocity;
        ctrl_cmd_pub_.publish(ctrl_cmd);
    }

    // 경로 생성
    path_ = generateSemiCircularPath(current_pos, goal_person, 100);

    ros::Rate rate(10);  // 10 Hz로 루프 실행
    
    while(!check_collision_success() && ros::ok()){
        publishPath(path_);
        //Spot current_pos = {current_x_, current_y_};

        // // ROS_INFO("DFSDF");

        // std::cout << current_x_ << std::endl;
        
        // Spot target_wpt = goal_person;
        // float diff_yaw = M_PI_2;

        followPath();
        // if( calculateDistance(current_pos, wpt) < 0.4){

        // }
        ros::spinOnce();


        // float lin_vel = 1.0;
        // float ang_vel = diff_yaw*0.3;

        // if(diff_yaw > M_PI_2*0.3)
        //     lin_vel = 0;

        // morai_msgs::SkidSteer6wUGVCtrlCmd ctrl_cmd;

        // ctrl_cmd.cmd_type = 3;
        // ctrl_cmd.Target_linear_velocity = lin_vel;
        // ctrl_cmd.Target_angular_velocity = ang_vel;

        // ctrl_cmd_pub_.publish(ctrl_cmd);
        rate.sleep();
    }
    result.success = true;

    PCAserver_.setSucceeded(result);
}

// 상태별 함수: FOLLOW_PATH
void person_action_node::followPath() {
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

    //ROS_INFO("Speed: %.2f, Angular Velocity: %.2f", speed, target_angular_velocity);

    // // 제어 명령 발행
    // morai_msgs::SkidSteer6wUGVCtrlCmd ctrl_cmd;
    // ctrl_cmd.cmd_type = 3;
    // ctrl_cmd.Target_linear_velocity = speed;
    // ctrl_cmd.Target_angular_velocity = target_angular_velocity;
    // ctrl_cmd_pub_.publish(ctrl_cmd);
    // ROS_INFO("Publishing Command: Linear Velocity = %.2f, Angular Velocity = %.2f, cmd_type = %d", speed, target_angular_velocity, ctrl_cmd.cmd_type);

    if(cnt%2 == 0){
        // 제어 명령 발행
        morai_msgs::SkidSteer6wUGVCtrlCmd ctrl_cmd;
        ctrl_cmd.cmd_type = 3;
        ctrl_cmd.Target_linear_velocity = 0;//speed;
        ctrl_cmd.Target_angular_velocity = target_angular_velocity;
        ctrl_cmd_pub_.publish(ctrl_cmd);
    }
    else{
        morai_msgs::SkidSteer6wUGVCtrlCmd ctrl_cmd;
        ctrl_cmd.cmd_type = 3;
        ctrl_cmd.Target_linear_velocity = speed;
        ctrl_cmd.Target_angular_velocity = 0;//target_angular_velocity;
        ctrl_cmd_pub_.publish(ctrl_cmd);
    }

    cnt++;
}


// 스티어링 각도 계산 함수
double person_action_node::steering_angle() {
    bool is_look_forward_point = false;
    geometry_msgs::Point rotated_point;
    double lfd = 0.3;
    double steering;

    // 뒤에서부터 경로점 순회
    for (int i = path_.size() - 1; i >= 0; --i) {
        const auto& wp = path_[i];

        // 좌표 변환: 로봇 기준으로 경로점 위치 변환
        double dx = wp.x - current_x_;
        double dy = wp.y - current_y_;

        rotated_point.x = cos(current_yaw_) * dx + sin(current_yaw_) * dy;
        rotated_point.y = sin(current_yaw_) * dx - cos(current_yaw_) * dy;

        if (rotated_point.x > 0) {
            double dis = sqrt(pow(rotated_point.x, 2) + pow(rotated_point.y, 2));
            if (dis >= lfd) {
                //forward_point = wp;
                is_look_forward_point = true;
                break;
            }
        }
    }

    // 스티어링 각도 계산
    if (is_look_forward_point) {
        double theta = atan2(rotated_point.y, rotated_point.x);
        steering = atan2(2 * 0.39 * sin(theta), lfd) * 180.0 / M_PI;
    } else {
        steering = 0;
    }
    return steering;
}



// 곡률 계산 함수
double person_action_node::calculateCurvature() {
    if (path_.size() < 3) {
        return 0.0;
    }

    // 현재 위치에 가장 가까운 웨이포인트 인덱스 찾기
    size_t idx = 0;
    double min_dist = std::numeric_limits<double>::max();

    for (size_t i = 0; i < path_.size(); ++i) {
        double dist = hypot(path_[i].x - current_x_, path_[i].y - current_y_);
        if (dist < min_dist) {
            min_dist = dist;
            idx = i;
        }
    }

    if (idx + 2 >= path_.size()) {
        return 0.0;
    }

    // 세 개의 웨이포인트 좌표
    double x1 = path_[idx].x;
    double y1 = path_[idx].y;
    double x2 = path_[idx + 1].x;
    double y2 = path_[idx + 1].y;
    double x3 = path_[idx + 2].x;
    double y3 = path_[idx + 2].y;

    // 곡률 계산 (kappa)
    double kappa = fabs((x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1)) /
                   (pow((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1), 1.5) + 1e-6);
    return kappa;
}

void person_action_node::publishPath(const std::vector<Spot>& path) {
    nav_msgs::Path path_msg;
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = "map";  // map 프레임 사용

    for (const auto& waypoint : path) {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "map";  // map 프레임 사용
        pose.pose.position.x = waypoint.x;
        pose.pose.position.y = waypoint.y;
        pose.pose.position.z = 0;  // 2D 경로이므로 z는 0으로 설정
        pose.pose.orientation.w = 1.0;  // 회전은 설정하지 않음 (2D에서 필요 없음)

        path_msg.poses.push_back(pose);  // 경로에 점 추가
    }

    // 경로 퍼블리시
    path_pub_.publish(path_msg);
}

void person_action_node::currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    // x, y 위치 업데이트
    current_x_ = msg->pose.position.x;
    current_y_ = msg->pose.position.y;
    
    // 쿼터니언을 이용해 yaw 값 추출
    tf::Quaternion q(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w
    );
    
    // tf::Quaternion을 tf::Matrix3x3로 변환하여 roll, pitch, yaw를 추출
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);  // roll, pitch, yaw 계산

    // 현재 yaw 값을 저장
    current_yaw_ = yaw;

    // 출력 예시 (디버깅용)
    //ROS_INFO("Current Position: (x: %f, y: %f), Yaw: %f", current_x_, current_y_, current_yaw_);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "person_collision_action_node2");
    ros::NodeHandle nh;

    person_action_node PAN(nh);

    ros::spin();
    return 0;
}