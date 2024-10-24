#include "Morai_Woowa/person_action_node.h"

person_action_node::person_action_node(ros::NodeHandle& nh):
PCAserver_(nh, "person_collision_action2", boost::bind(&person_action_node::execute, this, _1), false),
generator()
{
    PCAserver_.start(); // 액션 서버 시작
    lidar_utm_sub = nh.subscribe("/lidar_utm", 10, &person_action_node::pointcloud_callback, this);
    current_pose_sub_ = nh.subscribe("/current_pose", 10, &person_action_node::currentPoseCallback, this);

    ctrl_cmd_pub_ = nh.advertise<morai_msgs::SkidSteer6wUGVCtrlCmd>("/6wheel_skid_ctrl_cmd", 10);
    astar_path_pub_ = nh.advertise<nav_msgs::Path>("/astar_path", 10);
    ld_marker_pub_= nh.advertise<visualization_msgs::Marker>("/astar_ld", 10);
    goal_marker_pub_= nh.advertise<visualization_msgs::Marker>("/astar_goal", 10);
    rviz_pointcloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("astar_points_cloud", 1);

    current_x_ = 0;
    current_y_ = 0;

    last_target_found_time_ = ros::Time::now();

    world_size_limit_ = 5;
    grid_size_ = 0.1;

    erase_obs_from_person_dis_ = 0.9;

    generator.setHeuristic(AStar::Heuristic::euclidean);
    generator.setBackMovement(true);
    generator.setGridSize(grid_size_);
    generator.setCollisionDis(0.4);
    generator.setCollisionDis_sub(0.2);
}

bool person_action_node::check_collision_success(){

    float dis = 10000;
    
    Spot respawn_spot_indoor(430.00, -140.00);// 실내 리스폰 지점
    dis = calculateDistance(current_x_, current_y_, respawn_spot_indoor.x, respawn_spot_indoor.y);        

    if (dis < 0.5){
        ROS_WARN("indoor teleport success");
        return true;
    }

    Spot respawn_spot_outdoor(430.00, -140.00);// 야외 리스폰 지점
    dis = calculateDistance(current_x_, current_y_, respawn_spot_outdoor.x, respawn_spot_outdoor.y);        

    if (dis < 0.5){
        ROS_WARN("outdoor teleport success");
        return true;
    }

    else{
        ROS_WARN("teleport fail");
        return false;
    }
}

void person_action_node::back_move(float dur_t){
    auto t = ros::Time::now();
    while(ros::Time::now()-t < ros::Duration(dur_t)){
        morai_msgs::SkidSteer6wUGVCtrlCmd ctrl_cmd;
        ctrl_cmd.cmd_type = 3;
        ctrl_cmd.Target_linear_velocity = -1.0;
        ctrl_cmd.Target_angular_velocity = 0;//target_angular_velocity;
        ctrl_cmd_pub_.publish(ctrl_cmd);
    }
}

void person_action_node::stop(float dur_t){
    auto t = ros::Time::now();
    while(ros::Time::now()-t < ros::Duration(dur_t)){
        morai_msgs::SkidSteer6wUGVCtrlCmd ctrl_cmd;
        ctrl_cmd.cmd_type = 3;
        ctrl_cmd.Target_linear_velocity = 0.0;
        ctrl_cmd.Target_angular_velocity = 0;//target_angular_velocity;
        ctrl_cmd_pub_.publish(ctrl_cmd);
    }
}

AStar::Vec2i person_action_node::load_spot(int spot, bool is_indoor){
    
    AStar::Vec2i person_spot;
    
    if(is_indoor){
        if(spot == 4)
            person_spot = {-77, 7};
        else if(spot == 5)
            person_spot = {30.8, -42.4};

    }

    return person_spot;
}

void person_action_node::execute(const morai_woowa::Person_Collision_Act2GoalConstPtr& goal)
{
    ROS_INFO("Execute action: Person Collision Action!");

    morai_woowa::Person_Collision_Act2Feedback feedback;
    morai_woowa::Person_Collision_Act2Result result;

    generator.setWorldSize(world_x_limit_, world_y_limit_);

    goal_person_ = load_spot(goal->spot, goal->is_indoor);

    float grid_goal_x_ = static_cast<int>(goal_person_.x / grid_size_);
    float grid_goal_y_ = static_cast<int>(goal_person_.y / grid_size_);
    float grid_goal_center_x = grid_goal_x_ * grid_size_ + grid_size_ / 2.0f;
    float grid_goal_center_y = grid_goal_y_ * grid_size_ + grid_size_ / 2.0f;

    back_move(1);
    
    while(ros::ok()){

        if( (grid_goal_center_x <= world_x_limit_.x || grid_goal_center_x >= world_x_limit_.y ||
            grid_goal_center_y <= world_y_limit_.x || grid_goal_center_y >= world_y_limit_.y))
        {
            ROS_WARN("TOO far !!");
            result.success = false;
            break;
        }

        ros::spinOnce();
        generator.clearCollisions();
        generator.addCollision(astar_point_vector_);

        float grid_current_x_ = static_cast<int>(current_x_ / grid_size_);
        float grid_current_y_ = static_cast<int>(current_y_ / grid_size_);
        float grid_current_center_x = grid_current_x_ * grid_size_ + grid_size_ / 2.0f;
        float grid_current_center_y = grid_current_y_ * grid_size_ + grid_size_ / 2.0f;

        pub_marker_astar_goal(grid_goal_center_x, grid_goal_center_y);

        auto path = generator.findPath({grid_current_center_x, grid_current_center_y}, {grid_goal_center_x, grid_goal_center_y});
        
        int path_size = path.size();

        if (calculateDistance(path[path_size-1].x, path[path_size-1].y, grid_goal_center_x, grid_goal_center_y)> 1 ){
            ROS_WARN("Worng path were generated!!");
            back_move(0.5);
            continue;
        }

        nav_msgs::Path path_msg;
        path_msg.header.frame_id = "map"; 
        for(auto& coordinate : path) {
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.pose.position.x = coordinate.x;
            pose.pose.position.y = coordinate.y;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.w = 1.0;
            path_msg.poses.push_back(pose);
            }
        astar_path_pub_.publish(path_msg);
        

        int ld_index = std::max(path_size - 10, 0);
        auto LD_point = path[ld_index];
        float LD_x = LD_point.x;
        float LD_y = LD_point.y;
        pub_marker_astar_LD(LD_x, LD_y);

        // feedback.dis = is_target;
        
        // PCAserver_.publishFeedback(feedback);

        double ly = atan2(LD_y - current_y_, LD_x - current_x_);

        double d_yaw = ly - current_yaw_;

        d_yaw = (d_yaw > M_PI)? d_yaw - 2*M_PI : d_yaw;
        d_yaw = (d_yaw < -M_PI)? d_yaw + 2*M_PI : d_yaw;

        std::cout << d_yaw << " : d_yaw" << std::endl; 
        std::cout << ly << " : l_yaw" << std::endl; 
        std::cout << current_yaw_ << " : current_yaw_" << std::endl; 
        std::cout << path_size << " : path size" << std::endl; 


        float lin_vel = 0.1;
        float ang_vel = 0.0;

        if(abs(d_yaw) < 15*M_PI/180){ 
        lin_vel = 2.0;
        ang_vel = 0.0;
        }
        else{
            // lin_vel = 0.0;
            // ang_vel = 0.4;
            if(d_yaw > 0){
            lin_vel = 0.0;
            ang_vel = -0.3;
            }
            else {
            lin_vel = 0.0;
            ang_vel = 0.3;
            }
        }
        
        morai_msgs::SkidSteer6wUGVCtrlCmd ctrl_cmd;
        ctrl_cmd.cmd_type = 3;
        ctrl_cmd.Target_linear_velocity = lin_vel;
        ctrl_cmd.Target_angular_velocity = ang_vel;
        ctrl_cmd_pub_.publish(ctrl_cmd);

        if (check_collision_success()){
            result.success = true;
            stop(0.5)
            break;
        }

    }
    
    result.success = false;

    PCAserver_.setSucceeded(result);
}

void person_action_node::pub_marker_astar_LD(float LD_x, float LD_y)
{
    // 마커 메시지 생성
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.ns = "ld_point";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = LD_x;
    marker.pose.position.y = LD_y;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;  // 크기 설정
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;  // 불투명도 설정
    marker.color.r = 1.0;  // 빨간색
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    // 마커 퍼블리시
    ld_marker_pub_.publish(marker);
}

void person_action_node::pub_marker_astar_goal(float goal_x, float goal_y)
{
    // 마커 메시지 생성
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.ns = "ld_point";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = goal_x;
    marker.pose.position.y = goal_y;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;  // 크기 설정
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;  // 불투명도 설정
    marker.color.r = 1.0;  // 빨간색
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    // 마커 퍼블리시
    goal_marker_pub_.publish(marker);
}

void person_action_node::currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_x_ = msg->pose.position.x;
    current_y_ = msg->pose.position.y;

    tf::Quaternion q(
    msg->pose.orientation.x,
    msg->pose.orientation.y,
    msg->pose.orientation.z,
    msg->pose.orientation.w);

    tf::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, current_yaw_);  // Yaw 값을 얻음 (라디안 단위)
    
    world_x_limit_ = {current_x_ - world_size_limit_, current_x_ + world_size_limit_};
    world_y_limit_ = {current_y_ - world_size_limit_, current_y_ + world_size_limit_};
}

void person_action_node::pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    astar_point_vector_.clear(); // 이전 데이터 비우기

    // pcl::PointCloud 객체를 생성합니다.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    // sensor_msgs::PointCloud2를 pcl::PointCloud<pcl::PointXYZ>로 변환합니다.
    pcl::fromROSMsg(*msg, *cloud);

    // 중복을 피하기 위한 set
    std::set<std::pair<float, float>> unique_points;
    // 새로운 PointCloud2 메시지 생성
    sensor_msgs::PointCloud2 output_cloud;
    pcl::PointCloud<pcl::PointXYZ> pcl_output_cloud;
    // 포인트 클라우드를 XY 평면에 프로젝션하고 제한된 영역으로 잘라냅니다.
    for (const auto& point : cloud->points) 
    {
        // 현재 위치와 제한된 영역을 기준으로 포인트를 필터링합니다.
        if (point.x >= world_x_limit_.x && point.x <= world_x_limit_.y &&
            point.y >= world_y_limit_.x && point.y <= world_y_limit_.y && point.z > -0.6)
        {
            float grid_current_x_ = static_cast<int>(point.x / grid_size_);
            float grid_current_y_ = static_cast<int>(point.y / grid_size_);
            float grid_current_center_x = grid_current_x_ * grid_size_ + grid_size_ / 2.0f;
            float grid_current_center_y = grid_current_y_ * grid_size_ + grid_size_ / 2.0f;

            // 중복 체크
            std::pair<float, float> temp_point = {grid_current_center_x, grid_current_center_y};
            if (unique_points.insert(temp_point).second) {
                
                // 중복되지 않을 경우만 astar_point_vector_에 추가
                if(calculateDistance(grid_current_center_x, grid_current_center_y, goal_person_.x, goal_person_.y) < erase_obs_from_person_dis_)
                    continue;

                astar_point_vector_.push_back({grid_current_center_x, grid_current_center_y});

                // PointCloud에 추가
                pcl::PointXYZ pcl_point;
                pcl_point.x = grid_current_center_x;
                pcl_point.y = grid_current_center_y;
                pcl_point.z = 0.0; // Z 축은 0으로 설정
                pcl_output_cloud.points.push_back(pcl_point);
            }
        }
    }

    // pcl::PointCloud를 sensor_msgs::PointCloud2로 변환
    pcl::toROSMsg(pcl_output_cloud, output_cloud);
    output_cloud.header.frame_id = "map";  // Frame 설정
    output_cloud.header.stamp = ros::Time::now();

    // 포인트 클라우드를 퍼블리시
    rviz_pointcloud_pub_.publish(output_cloud);

    return;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "person_collision_action_node");
    ros::NodeHandle nh;

    person_action_node PAN(nh);

    ros::spin();
    return 0;
}