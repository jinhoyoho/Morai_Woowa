#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/icp.h>
#include <vector>
#include <std_msgs/Header.h>
#include <cmath>

pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr ex_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZ>::Ptr centered_point_ptr(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr translation_ptr(new pcl::PointCloud<pcl::PointXYZ>());
auto centered_point_msg_ptr = std::make_shared<sensor_msgs::PointCloud2>();
auto arrow_array_ptr = std::make_shared<visualization_msgs::MarkerArray>();
auto clusters_ptr = std::make_shared<std::vector<pcl::PointCloud<pcl::PointXYZI>>>();

void cluster_callback(const sensor_msgs::PointCloud2::ConstPtr& msg);
void tracking();
void make_arrows();
void removeSmallClusters();

int main(int argc, char **argv){
    ros::init(argc, argv, "obstacle_tracking");
    ros::NodeHandle nh;
    ros::Subscriber cluster_sub = nh.subscribe("clustered_points", 10, cluster_callback);
    ros::Publisher center_pub = nh.advertise<sensor_msgs::PointCloud2>("center", 10);
    ros::Publisher marker_array_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);
    ros::Rate loop_rate(10);

    while (ros::ok()) {
        pcl::toROSMsg(*centered_point_ptr, *centered_point_msg_ptr);
        // centered_point_msg_ptr->header.stamp = ros::Time::now();
        // centered_point_msg_ptr->header.frame_id = "map";
        // center_pub.publish(*centered_point_msg_ptr);
        marker_array_pub.publish(*arrow_array_ptr);
        loop_rate.sleep();
        // ROS_INFO("%ld", clusters_ptr->size());
        ros::spinOnce();
    }
    return 0;
}

void cluster_callback(const sensor_msgs::PointCloud2::ConstPtr& msg){
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

    tracking();

    *ex_cloud_ptr = *cloud_ptr;

    make_arrows();
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

void tracking(){
    pcl::PassThrough<pcl::PointXYZI> pass;
    pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
    icp.setMaxCorrespondenceDistance(1.0);  // 점군 간의 최대 대응 거리 (기본값보다 크게 설정)
    icp.setTransformationEpsilon(1e-1);    // 변환 허용 오차
    icp.setMaximumIterations(50); 
    pass.setInputCloud(ex_cloud_ptr);

    centered_point_ptr->clear();
    centered_point_ptr->resize(clusters_ptr->size());
    translation_ptr->resize(clusters_ptr->size());

    //get center
    for(int i=0; i<clusters_ptr->size(); i++){
        pcl::PointCloud<pcl::PointXYZI>::Ptr current_cluster_ptr = clusters_ptr->at(i).makeShared();
        auto mean_x = 0.0;
        auto mean_y = 0.0;
        auto max_x = -999.9;
        auto min_x = 999.9;
        auto max_y = -999.9;
        auto min_y = 999.9;

        for(int j=0; j<current_cluster_ptr->size(); j++){
            mean_x += current_cluster_ptr->at(j).x;
            mean_y += current_cluster_ptr->at(j).y;

            if(current_cluster_ptr->at(j).y > max_y){
                max_y = current_cluster_ptr->at(j).y;
            }
            else if(current_cluster_ptr->at(j).y < min_y){
                min_y = current_cluster_ptr->at(j).y;
            }

            if(current_cluster_ptr->at(j).x > max_x){
                max_x = current_cluster_ptr->at(j).x;
            }
            else if(current_cluster_ptr->at(j).x < min_x){
                min_x = current_cluster_ptr->at(j).x;
            }
        }
        mean_x = mean_x/current_cluster_ptr->size();
        mean_y = mean_y/current_cluster_ptr->size();
        pcl::PointXYZ point;
        point.x = mean_x;
        point.y = mean_y;
        point.z = 0.0;
        centered_point_ptr->at(i) = point;

        //cutting map
        pcl::PointCloud<pcl::PointXYZI>::Ptr cutted_map_ptr(new pcl::PointCloud<pcl::PointXYZI>());
        pass.setFilterFieldName("y");
        pass.setFilterLimits(-1 + min_y, 1 + max_y);
        pass.filter(*cutted_map_ptr);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(-1 + min_x, 1 + max_x);
        pass.filter(*cutted_map_ptr);

        //icp
        icp.setInputSource(current_cluster_ptr);
        icp.setInputTarget(cutted_map_ptr);
        pcl::PointCloud<pcl::PointXYZI> Final; 
        icp.align(Final);
        float t_x = 0.0;
        float t_y = 0.0;
        if (icp.hasConverged()) {
            Eigen::Matrix4f transformation = icp.getFinalTransformation();
            t_x = transformation(0, 3);
            t_y = transformation(1, 3);
        }

        translation_ptr->at(i).x = t_x;
        translation_ptr->at(i).y = t_y;
        translation_ptr->at(i).z = 0;
    }
}

void make_arrows(){
    arrow_array_ptr->markers.clear();
    for(int i=0; i<translation_ptr->size(); i++){
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
            start_point.x = centered_point_ptr->at(i).x;  // X축을 따라 화살표가 멀어지도록 설정
            start_point.y = centered_point_ptr->at(i).y;
            start_point.z = 0.0;

            geometry_msgs::Point end_point;
            end_point.x = centered_point_ptr->at(i).x + 10 * translation_ptr->at(i).x;
            end_point.y = centered_point_ptr->at(i).y + 10 * translation_ptr->at(i).y;
            end_point.z = 0.0;

            // 화살표 크기 설정
            marker.scale.x = 0.1;  // 화살표의 길이
            marker.scale.y = 0.2;  // 화살표의 두께
            marker.scale.z = 0.2;  // 화살표 머리의 크기

            // 화살표 색상 설정 (RGBA 형식)
            marker.color.r = 1.0f;  // 점점 색이 변하도록 설정
            marker.color.g = 0.2f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0f;  // 불투명

            marker.points.push_back(start_point);
            marker.points.push_back(end_point);

            arrow_array_ptr->markers.push_back(marker);
    }
}