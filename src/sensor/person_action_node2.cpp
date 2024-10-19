#include "Morai_Woowa/person_action_node2.h"

person_action_node::person_action_node(ros::NodeHandle& nh):PCAserver_(nh, "person_collision_action2", boost::bind(&person_action_node::execute, this, _1), false)
{
    PCAserver_.start(); // 액션 서버 시작
    current_pose_sub_ = nh.subscribe("/current_pose", 10, &person_action_node::currentPoseCallback, this);

    ctrl_cmd_pub_ = nh.advertise<morai_msgs::SkidSteer6wUGVCtrlCmd>("/6wheel_skid_ctrl_cmd", 10);

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
    
    std::vector<Spot> path;
    
    // 시작점과 목표점 사이의 중간점 계산
    Spot mid;
    mid.x = (start.x + goal.x) / 2.0;
    mid.y = (start.y + goal.y) / 2.0;

    double direction = atan2(goal.y - start.y, goal.x - start.x);

    // 반지름 계산 (중간점에서 시작점/목표점까지의 거리)
    double radius = calculateDistance(mid, start);

    // 반원을 그리기 위한 각도 계산 (반원을 적당히 나누기 위한 각도)
    double theta_step = M_PI / num_points;  // 반원을 num_points로 나눔

    // 반원 경로의 점 계산
    for (int i = 0; i <= num_points; ++i) {
        Spot p;
        double theta = i * theta_step;

        // 좌표계의 반원 방정식을 이용한 계산
        p.x = mid.x + radius * std::cos(- direction + theta);  // 중심에서 x 좌표 계산
        p.y = mid.y + radius * std::sin(- direction + theta);  // 중심에서 y 좌표 계산

        path.push_back(p);
    }

    return path;
}


Spot person_action_node::load_spot(int spot, bool is_indoor){
    Spot indoor_5(30.44, -41.68);
    return indoor_5;
}

void person_action_node::execute(const morai_woowa::Person_Collision_Act2GoalConstPtr& goal)
{
    morai_woowa::Person_Collision_Act2Feedback feedback;
    morai_woowa::Person_Collision_Act2Result result;

    ROS_INFO("Execute action: Person Collision Action!");

    Spot goal_person = load_spot(goal->spot, goal->is_indoor);

    // 현재 위치와 목표 위치 설정
    Spot current_pos = {current_x_, current_y_};

    // 경로 생성
    std::vector<Spot> path = generateSemiCircularPath(current_pos, goal_person, 20);

    while(check_collision_success() && ros::ok()){
        
        Spot target_wpt = goal_person;
        float diff_yaw = M_PI_2;

        for(auto wpt : path){
            float dis = calculateDistance(current_pos, wpt);
            float direction = atan2(wpt.y - current_pos.y, wpt.x - current_pos.x);
            diff_yaw = direction - current_yaw_;

            if(dis < 0.3 && fabs(diff_yaw) < M_PI_2){
                target_wpt = wpt;  
                break;              
            }
        }

        float lin_vel = 2.0;
        float ang_vel = diff_yaw*0.5;
        morai_msgs::SkidSteer6wUGVCtrlCmd ctrl_cmd;

        ctrl_cmd.cmd_type = 3;
        ctrl_cmd.Target_linear_velocity = lin_vel;
        ctrl_cmd.Target_angular_velocity = ang_vel;

        ctrl_cmd_pub_.publish(ctrl_cmd);
    }
    result.success = true;

    PCAserver_.setSucceeded(result);
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
    ROS_INFO("Current Position: (x: %f, y: %f), Yaw: %f", current_x_, current_y_, current_yaw_);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "person_collision_action_node2");
    ros::NodeHandle nh;

    person_action_node PAN(nh);

    ros::spin();
    return 0;
}