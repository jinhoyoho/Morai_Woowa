#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <vector>
#include <std_msgs/Header.h>
#include <cmath>

//parameters

// DBSCAN Parameters
const double EPSILON = 1;  // Minimum distance between points
const int MIN_POINTS = 3;    // Minimum number of points required to form a cluster


// functions
void lidar_callback(const sensor_msgs::PointCloud2::ConstPtr& msg);
void lidar_processing();
void roi();
void make_2d();
void voxel();
std::vector<int> dbscan(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double epsilon, int minPoints);

//variables
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZI>::Ptr processed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
sensor_msgs::PointCloud2 output_msg;


int main(int argc, char **argv){
    ros::init(argc, argv, "lidar_db");
    ros::NodeHandle nh;
    ros::Subscriber lidar_sub = nh.subscribe("velodyne_points", 10, lidar_callback);
    ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("clustered_points", 1);
    ros::Rate loop_rate(10);

    while (ros::ok()) {
        output_msg.header.stamp = ros::Time::now();
        output_msg.header.frame_id = "map";
        cloud_pub.publish(output_msg);
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}

void lidar_callback(const sensor_msgs::PointCloud2::ConstPtr& msg){ 
    pcl::fromROSMsg(*msg, *cloud_ptr);
    lidar_processing();
}

void lidar_processing(){
    roi();
    // make_2d();
    voxel();
    std::vector<int> cluster = dbscan(cloud_ptr, EPSILON, MIN_POINTS);

    //allocate
    processed_cloud_ptr->points.resize(cloud_ptr->points.size());
    for(int i=0; i<cluster.size(); i++){
        processed_cloud_ptr->at(i).x = cloud_ptr->at(i).x;
        processed_cloud_ptr->at(i).y = cloud_ptr->at(i).y;
        processed_cloud_ptr->at(i).z = cloud_ptr->at(i).z;
        processed_cloud_ptr->at(i).intensity = cluster.at(i);
    }
    pcl::toROSMsg(*processed_cloud_ptr, output_msg);
}

void roi(){
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_ptr);

        pass.setFilterFieldName("z");
        pass.setFilterLimits(-0.5, 1.0);
        pass.filter(*cloud_ptr);

        pass.setFilterFieldName("y");
        pass.setFilterLimits(-3, 3);
        pass.filter(*cloud_ptr);

        pass.setFilterFieldName("x");
        pass.setFilterLimits(-0, 10);
        pass.filter(*cloud_ptr);
}

void make_2d(){
    for (auto& point : cloud_ptr->points)
    {
        point.z = 0.0;
    }   
}

void voxel(){
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setLeafSize(0.05, 0.05, 0.05);
    sor.setInputCloud(cloud_ptr);
    sor.filter(*cloud_ptr);
}

// 포인트 간의 유클리드 거리 계산 함수
double euclideanDistance(const pcl::PointXYZ& point1, const pcl::PointXYZ& point2) {
    return std::sqrt(std::pow(point1.x - point2.x, 2) +
                     std::pow(point1.y - point2.y, 2) +
                     std::pow(point1.z - point2.z, 2));
}

// DBSCAN 알고리즘
std::vector<int> dbscan(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double epsilon, int minPoints) {
    int numPoints = cloud->size();
    std::vector<bool> visited(numPoints, false);    // 방문 여부 체크
    std::vector<int> cluster(numPoints, -1);        // 클러스터 번호, -1은 노이즈
    int clusterID = 0;

    for (int i = 0; i < numPoints; ++i) {
        if (visited[i]) continue;

        visited[i] = true;
        std::vector<int> neighbors;

        // 이웃 포인트 찾기
        for (int j = 0; j < numPoints; ++j) {
            if (euclideanDistance(cloud->points[i], cloud->points[j]) <= epsilon) {
                neighbors.push_back(j);
            }
        }

        // 이웃 포인트가 충분하지 않으면 노이즈로 간주
        if (neighbors.size() < minPoints) {
            cluster[i] = -1;  // 노이즈
        } else {
            // 새로운 클러스터 생성
            clusterID++;
            cluster[i] = clusterID;

            // 이웃 포인트들 처리
            for (int k = 0; k < neighbors.size(); ++k) {
                int neighborIndex = neighbors[k];
                if (!visited[neighborIndex]) {
                    visited[neighborIndex] = true;

                    std::vector<int> neighborNeighbors;
                    // 새로운 이웃 포인트들을 찾음
                    for (int j = 0; j < numPoints; ++j) {
                        if (euclideanDistance(cloud->points[neighborIndex], cloud->points[j]) <= epsilon) {
                            neighborNeighbors.push_back(j);
                        }
                    }

                    if (neighborNeighbors.size() >= minPoints) {
                        neighbors.insert(neighbors.end(), neighborNeighbors.begin(), neighborNeighbors.end());
                    }
                }

                if (cluster[neighborIndex] == -1) {
                    cluster[neighborIndex] = clusterID;
                }
            }
        }
    }

    return cluster;
}

