#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <vector>
#include <std_msgs/Header.h>
#include <cmath>

//2024 09 29 원래 icp로 장애물 추적하려고 했는데 장애물 모양이 계속 달라져서 추적하기 어려움 -> 다시 확인해보니 추적은 되는데 한 프레임만으로 움직임을 정하는게 어려움, 오차가 꽤 큼. 그래서 10프레임을 모아서 정해야겠다
//2024 10 01 obstacle topic에 장애물 정보 publish x:x좌표, y:y좌표, z:x방향속도, i:y방향속도 + tracking 초기에 낮은 속도가 나오던 버그 수정

pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZ>::Ptr center_points_ptr(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr translation_ptr(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZI>::Ptr obstacle_ptr(new pcl::PointCloud<pcl::PointXYZI>());

auto cloud_msg_ptr = std::make_shared<sensor_msgs::PointCloud2>();
auto obstacle_msg_ptr = std::make_shared<sensor_msgs::PointCloud2>();
auto arrow_array_ptr = std::make_shared<visualization_msgs::MarkerArray>();
auto clusters_ptr = std::make_shared<std::vector<pcl::PointCloud<pcl::PointXYZI>>>();
auto center_points_history_ptr = std::make_shared<std::vector<pcl::PointCloud<pcl::PointXYZ>>>();

void cluster_callback(const sensor_msgs::PointCloud2::ConstPtr& msg);
pcl::PointXYZ get_center(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr);
void tracking();
void make_arrows();
void make_obstacle_msg();
void removeSmallClusters();

int marker_id = 0;

int main(int argc, char **argv){
    ros::init(argc, argv, "obstacle_tracking");
    ros::NodeHandle nh;
    // ros::Subscriber cluster_sub = nh.subscribe("clustered_points", 10, cluster_callback);
    ros::Subscriber cluster_sub = nh.subscribe("lidar_utm", 10, cluster_callback);
    ros::Publisher marker_array_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);
    ros::Publisher obstacle_pub = nh.advertise<sensor_msgs::PointCloud2>("obstacle", 10);
    ros::Rate loop_rate(10);

    while (ros::ok()) {
        pcl::toROSMsg(*cloud_ptr, *cloud_msg_ptr);
        std::cout<<arrow_array_ptr->markers.size()<<std::endl;
        marker_array_pub.publish(*arrow_array_ptr);
        obstacle_pub.publish(*obstacle_msg_ptr);
        loop_rate.sleep();
        // ROS_INFO("%ld", clusters_ptr->size());
        ros::spinOnce();
    }
    return 0;
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
            make_arrows();
            make_obstacle_msg();
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

    // clusters_ptr->erase(
    //     std::remove_if(
    //         clusters_ptr->begin(), 
    //         clusters_ptr->end(),
    //         [](const pcl::PointCloud<pcl::PointXYZI>& cloud) {
    //             return cloud.size() >= 100;  // 사이즈가 10 이하인 클러스터를 제거
    //         }
    //     ), 
    //     clusters_ptr->end()
    // );
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
    translation_ptr->resize(center_points_history_ptr->at(9).size());
    
    for(int i=0; i<translation_ptr->size(); i++){
        auto search_point = center_points_history_ptr->at(9).at(i);
        for(int j=0; j<center_points_history_ptr->size() -1; j++){
            auto current_scene = center_points_history_ptr->at(9-j);
            auto target_scene = center_points_history_ptr->at(8-j);
            
            kdtree.setInputCloud(target_scene.makeShared());
            int K = 1;  // 최근접점 10개를 찾음
            std::vector<int> pointIdxNKNSearch(K);
            std::vector<float> pointNKNSquaredDistance(K);

            kdtree.nearestKSearch(search_point, K, pointIdxNKNSearch, pointNKNSquaredDistance);

            auto nearest_point = target_scene.at(pointIdxNKNSearch[0]);

            if(pointNKNSquaredDistance[0]<1){
                if(- nearest_point.x + search_point.x < 0.5){
                    translation_ptr->at(i).x += - nearest_point.x + search_point.x;
                }
                else{
                    translation_ptr->at(i).x = 0.0;
                }
                
                if(- nearest_point.y + search_point.y < 0.5){
                    translation_ptr->at(i).y += - nearest_point.y + search_point.y;  
                }
                else{
                    translation_ptr->at(i).y = 0.0;
                }
                 
                translation_ptr->at(i).z += 0.1;
                search_point = nearest_point;
            }
        }
    }

}


void make_arrows() {
    // 먼저 기존 마커들을 삭제
    for (int i = 0; i < arrow_array_ptr->markers.size(); i++) {
        visualization_msgs::Marker delete_marker;
        delete_marker.header.frame_id = "map";  // 'map' 좌표계 기준
        delete_marker.header.stamp = ros::Time::now();
        delete_marker.ns = "arrows";
        delete_marker.id = arrow_array_ptr->markers[i].id;  // 기존 마커의 id로 삭제
        delete_marker.action = visualization_msgs::Marker::DELETE;  // 마커 삭제 명령

        // 삭제 마커를 퍼블리시할 배열에 추가
        arrow_array_ptr->markers[i] = delete_marker;
    }
    
    // 마커를 초기화하고 새 마커들을 추가
    // arrow_array_ptr->markers.clear();  // 이전 마커 모두 지운 후 초기화

    for (int i = 0; i < translation_ptr->size(); i++) {
        visualization_msgs::Marker marker;

        // Marker의 프레임 설정
        marker.header.frame_id = "map";  // 'map' 좌표계 기준
        marker.header.stamp = ros::Time::now();

        // Marker의 네임스페이스와 ID 설정
        marker.ns = "arrows";
        marker.id = marker_id;  // 각 마커는 고유한 ID를 가져야 함
        marker_id++;

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
}


void make_obstacle_msg(){
    obstacle_ptr->clear();
    obstacle_ptr->resize(center_points_history_ptr->at(9).size());

    for(int i=0; i<center_points_history_ptr->at(9).size(); i++){
        obstacle_ptr->at(i).x = center_points_history_ptr->at(9).at(i).x;
        obstacle_ptr->at(i).y = center_points_history_ptr->at(9).at(i).y;
        obstacle_ptr->at(i).z = translation_ptr->at(i).x;
        obstacle_ptr->at(i).intensity = translation_ptr->at(i).y;
    }

    pcl::toROSMsg(*obstacle_ptr, *obstacle_msg_ptr);

    obstacle_msg_ptr->header.frame_id = "map";  // 'map' 좌표계 기준
    obstacle_msg_ptr->header.stamp = ros::Time::now();
}