#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <morai_woowa/average_points_array.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <vector>
#include <std_msgs/Header.h>
#include <cmath>
#include <tf/tf.h>                          
#include <Eigen/Dense>  

//2024 09 29 원래 icp로 장애물 추적하려고 했는데 장애물 모양이 계속 달라져서 추적하기 어려움 -> 다시 확인해보니 추적은 되는데 한 프레임만으로 움직임을 정하는게 어려움, 오차가 꽤 큼. 그래서 10프레임을 모아서 정해야겠다
//2024 10 01 obstacle topic에 장애물 정보 publish x:x좌표, y:y좌표, z:x방향속도, i:y방향속도 + tracking 초기에 낮은 속도가 나오던 버그 수정

pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZ>::Ptr center_points_ptr(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr translation_ptr(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZI>::Ptr obstacle_ptr(new pcl::PointCloud<pcl::PointXYZI>());
geometry_msgs::PoseStamped current_pose;

auto cloud_msg_ptr = std::make_shared<sensor_msgs::PointCloud2>();
auto obstacle_msg_ptr = std::make_shared<sensor_msgs::PointCloud2>();
auto arrow_array_ptr = std::make_shared<visualization_msgs::MarkerArray>();
auto clusters_ptr = std::make_shared<std::vector<pcl::PointCloud<pcl::PointXYZI>>>();
auto center_points_history_ptr = std::make_shared<std::vector<pcl::PointCloud<pcl::PointXYZ>>>();

void cluster_callback(const sensor_msgs::PointCloud2::ConstPtr& msg);
void person_callback(const morai_woowa::average_points_array::ConstPtr& msg);
void pose_callback(const geometry_msgs::PoseStamped &msg);
pcl::PointXYZ get_center(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr);
void tracking();
void make_arrows(ros::Publisher& marker_pub);
void make_obstacle_msg();
void removeSmallClusters();

int hz = 60;

int marker_size = 0;

int main(int argc, char **argv){
    ros::init(argc, argv, "obstacle_tracking");
    ros::NodeHandle nh;
    // ros::Subscriber cluster_sub = nh.subscribe("clustered_points", 10, cluster_callback);
    // ros::Subscriber cluster_sub = nh.subscribe("lidar_utm", 10, cluster_callback);
    ros::Subscriber person_sub = nh.subscribe("/average_points", 10, person_callback);
    ros::Subscriber pose_sub = nh.subscribe("current_pose", 10, pose_callback);
    
    ros::Publisher marker_array_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker_array", 10);
    ros::Publisher obstacle_pub = nh.advertise<sensor_msgs::PointCloud2>("obstacle", 10);
    ros::Rate loop_rate(hz);

    while (ros::ok()) {
        pcl::toROSMsg(*cloud_ptr, *cloud_msg_ptr);
        // std::cout<<arrow_array_ptr->markers.size()<<std::endl;

        loop_rate.sleep();
        // ROS_INFO("%ld", clusters_ptr->size());
        ros::spinOnce();
        if(center_points_history_ptr->size() == hz){
            make_arrows(marker_array_pub);
        }
        obstacle_pub.publish(*obstacle_msg_ptr);
    }
    return 0;
}

void pose_callback(const geometry_msgs::PoseStamped &msg)
{
    current_pose = msg;
}

void person_callback(const morai_woowa::average_points_array::ConstPtr& msg){
    center_points_ptr->clear();

    if(msg->points_array.size() != 0){
        // pcl::PointCloud<pcl::PointXYZ> people;
        (*center_points_ptr).resize(msg->points_array.size());
    
        for(int i=0; i<msg->points_array.size(); i++){
            (*center_points_ptr)[i].x = msg->points_array[i].average_x;
            (*center_points_ptr)[i].y = msg->points_array[i].average_y;
            (*center_points_ptr)[i].z = msg->points_array[i].average_z;
        }


        // 쿼터니언으로부터 회전 행렬 생성
        tf::Quaternion q(
            current_pose.pose.orientation.x,
            current_pose.pose.orientation.y,
            current_pose.pose.orientation.z,
            current_pose.pose.orientation.w
        );

        tf::Matrix3x3 rotation_matrix(q);
        
        // 변환 행렬을 수동으로 설정
        Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                transform(i, j) = rotation_matrix[i][j];
            }
        }

        // 위치 값 설정
        // transform(0, 3) = static_cast<double>(current_pose.pose.position.x);
        // transform(1, 3) = static_cast<double>(current_pose.pose.position.y);
        // transform(2, 3) = static_cast<double>(current_pose.pose.position.z);

        // 포인트 클라우드 변환
        for (auto& point : center_points_ptr->points) {
            point.x += 0.42;

            // 회전 변환
            auto x = transform(0, 0) * point.x + transform(0, 1) * point.y + transform(0, 2) * point.z + current_pose.pose.position.x;//transform(0, 3);
            auto y = transform(1, 0) * point.x + transform(1, 1) * point.y + transform(1, 2) * point.z + current_pose.pose.position.y;//transform(1, 3);
            auto z = transform(2, 0) * point.x + transform(2, 1) * point.y + transform(2, 2) * point.z + current_pose.pose.position.z;//transform(2, 3);

            // 변환된 포인트를 클라우드에 추가
            point.x = x;
            point.y = y;
            point.z = z;
        }


        center_points_history_ptr->push_back((*center_points_ptr));

        if(center_points_history_ptr->size() > hz){
            center_points_history_ptr->erase(center_points_history_ptr->begin());
        }

        // std::cout<<center_points_history_ptr->size()<<"\n";

        if(center_points_history_ptr->size() == hz){
            tracking();
            make_obstacle_msg();
        }
    }
}

void cluster_callback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    if(msg->data.size() != 0){

        pcl::fromROSMsg(*msg, *cloud_ptr);


        clusters_ptr->clear();
        //classify
        for(int i=0; i<cloud_ptr->size(); i++){
            auto cluster_name = cloud_ptr->at(i).intensity;
            if(cluster_name != -1){
                if(cluster_name >= clusters_ptr->size()){
                    clusters_ptr->resize(cluster_name + 1);
                }
                clusters_ptr->at(cluster_name).push_back(cloud_ptr->at(i));
            }
        }
        removeSmallClusters();

        center_points_ptr->clear();
        center_points_ptr->resize(clusters_ptr->size());

        if(clusters_ptr->size()>0){

            // 중심 구하기
            for(int i=0; i<clusters_ptr->size(); i++){
                auto center_point = get_center(clusters_ptr->at(i).makeShared());
                center_points_ptr->at(i) = center_point;
            }

            // 데이터 쌓기
            if(center_points_ptr->size()>0){
                center_points_history_ptr->push_back(*center_points_ptr);
            }
            
            if(center_points_history_ptr->size() > 10){
                center_points_history_ptr->erase(center_points_history_ptr->begin());
            }

            if(center_points_history_ptr->size() == 10){
                tracking();
                make_obstacle_msg();
            }

        }
    }
}

void removeSmallClusters() {
    clusters_ptr->erase(
        std::remove_if(
            clusters_ptr->begin(), 
            clusters_ptr->end(),
            [](const pcl::PointCloud<pcl::PointXYZI>& cloud) {
                return cloud.size() <= 10;  // 사이즈가 10 이하인 클러스터를 제거
            }
        ), 
        clusters_ptr->end()
    );

    clusters_ptr->erase(
        std::remove_if(
            clusters_ptr->begin(), 
            clusters_ptr->end(),
            [](const pcl::PointCloud<pcl::PointXYZI>& cloud) {
                return cloud.size() >= 100;  // 사이즈가 10 이하인 클러스터를 제거
            }
        ), 
        clusters_ptr->end()
    );
}

pcl::PointXYZ get_center(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr){
    auto mean_x = 0.0;
    auto mean_y = 0.0;

    for(int j=0; j<cloud_ptr->size(); j++){
        mean_x += cloud_ptr->at(j).x;
        mean_y += cloud_ptr->at(j).y;
    }

    mean_x = mean_x/cloud_ptr->size();
    mean_y = mean_y/cloud_ptr->size();

    pcl::PointXYZ point;
    point.x = mean_x;
    point.y = mean_y;
    point.z = 0;

    return point;
}

void tracking(){
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    translation_ptr->clear();
    translation_ptr->resize(center_points_history_ptr->at(hz-1).size());
    
    for(int i=0; i<translation_ptr->size(); i++){
        auto search_point = center_points_history_ptr->at(hz-1).at(i);
        for(int j=0; j<center_points_history_ptr->size() -1; j++){
            auto current_scene = center_points_history_ptr->at(hz-1-j);
            auto target_scene = center_points_history_ptr->at(hz-2-j);
            
            kdtree.setInputCloud(target_scene.makeShared());
            int K = 1;  // 최근접점 10개를 찾음
            std::vector<int> pointIdxNKNSearch(K);
            std::vector<float> pointNKNSquaredDistance(K);

            kdtree.nearestKSearch(search_point, K, pointIdxNKNSearch, pointNKNSquaredDistance);

            auto nearest_point = target_scene.at(pointIdxNKNSearch[0]);
            // std::cout<<nearest_point<<std::endl;
            // std::cout<<search_point<<std::endl;
            // std::cout<<"----------"<<std::endl;
            if(pointNKNSquaredDistance[0]<1){
                if(- nearest_point.x + search_point.x < 0.5){
                    translation_ptr->at(i).x += - nearest_point.x + search_point.x;
                }
                // else{
                //     translation_ptr->at(i).x = 0.0;
                // }
                
                if(- nearest_point.y + search_point.y < 0.5){
                    translation_ptr->at(i).y += - nearest_point.y + search_point.y;  
                }
                // else{
                //     translation_ptr->at(i).y = 0.0;
                // }
                 
                translation_ptr->at(i).z += 1.0/(double)hz;
                search_point = nearest_point;
            }
        }
    }

}


void make_arrows(ros::Publisher& marker_pub) {
    // 기존 마커들을 삭제
    for (int i = 0; i < arrow_array_ptr->markers.size(); i++) {
        visualization_msgs::Marker delete_marker;
        delete_marker.header.frame_id = "map";  // 'map' 좌표계 기준
        delete_marker.header.stamp = ros::Time::now();
        delete_marker.ns = "arrows";
        delete_marker.id = arrow_array_ptr->markers[i].id;  // 기존 마커의 id로 삭제
        delete_marker.action = visualization_msgs::Marker::DELETE;  // 마커 삭제 명령

        // 삭제 마커를 퍼블리시할 배열에 추가
        marker_pub.publish(delete_marker);  // 삭제 마커 퍼블리시
    }
    
    // arrow_array_ptr->markers를 초기화하여 기존 마커 데이터를 삭제
    arrow_array_ptr->markers.clear();  

    // 마커를 초기화하고 새 마커들을 추가
    for (int i = 0; i < center_points_ptr->size(); i++) {
        visualization_msgs::Marker marker;

        // Marker의 프레임 설정
        marker.header.frame_id = "map";  // 'map' 좌표계 기준
        marker.header.stamp = ros::Time::now();

        // Marker의 네임스페이스와 ID 설정
        marker.ns = "arrows";
        marker.id = i;  // 각 마커는 고유한 ID를 가져야 함

        // 화살표 타입으로 설정
        marker.type = visualization_msgs::Marker::ARROW;

        // 화살표 동작 설정 (ADD는 새로운 화살표를 추가하거나 업데이트)
        marker.action = visualization_msgs::Marker::ADD;

        // 화살표 시작점과 끝점을 설정
        geometry_msgs::Point start_point;
        start_point.x = center_points_ptr->at(i).x;  // X축을 따라 화살표가 멀어지도록 설정
        start_point.y = center_points_ptr->at(i).y;
        start_point.z = 0.0;

        geometry_msgs::Point end_point;
        double tracking_time = translation_ptr->at(i).z;
        end_point.x = center_points_ptr->at(i).x + translation_ptr->at(i).x / tracking_time;
        end_point.y = center_points_ptr->at(i).y + translation_ptr->at(i).y / tracking_time;
        end_point.z = 0.0;

        // 화살표 크기 설정
        marker.scale.x = 0.1;  // 화살표의 길이
        marker.scale.y = 0.2;  // 화살표의 두께
        marker.scale.z = 0.2;  // 화살표 머리의 크기

        // 화살표 색상 설정 (RGBA 형식)
        marker.color.r = 1.0f;
        marker.color.g = 0.2f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;

        // 화살표 시작점과 끝점을 설정
        marker.points.push_back(start_point);
        marker.points.push_back(end_point);

        // 마커를 마커 배열에 추가
        arrow_array_ptr->markers.push_back(marker);
    }

    // 새 마커들을 퍼블리시
    for (int i = 0; i < arrow_array_ptr->markers.size(); i++) {
        marker_pub.publish(arrow_array_ptr->markers[i]);
    }
}



void make_obstacle_msg(){
    obstacle_ptr->clear();
    obstacle_ptr->resize(center_points_history_ptr->at(hz-1).size());

    for(int i=0; i<center_points_history_ptr->at(hz-1).size(); i++){
        obstacle_ptr->at(i).x = center_points_history_ptr->at(hz-1).at(i).x;
        obstacle_ptr->at(i).y = center_points_history_ptr->at(hz-1).at(i).y;
        obstacle_ptr->at(i).z = translation_ptr->at(i).x;
        obstacle_ptr->at(i).intensity = translation_ptr->at(i).y;
    }

    pcl::toROSMsg(*obstacle_ptr, *obstacle_msg_ptr);

    obstacle_msg_ptr->header.frame_id = "map";  // 'map' 좌표계 기준
    obstacle_msg_ptr->header.stamp = ros::Time::now();
}