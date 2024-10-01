#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <vector>
#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>

//2024 10 01 dwa가 동적 장애물을 받고 적용하도록 수정

int num_of_path = 15;
double predict_time = 3;
double velocity = 1.0;
double angular_velocity = 0.5;
double obstacle_cost = 2.0;
double global_path_cost = 0.1;

const double PI = 3.1415926;

void make_global_path();
double calculateDistance(const std::vector<double>& point1, const std::vector<double>& point2);
void obstacle_callback(const sensor_msgs::PointCloud2::ConstPtr& msg);
void make_candidate_path(std::shared_ptr<std::vector<std::vector<std::vector<double>>>> candidate_path_ptr);
void vote_president_path(std::shared_ptr<std::vector<std::vector<std::vector<double>>>> candidate_path_ptr);
void make_msg(std::shared_ptr<std::vector<std::vector<std::vector<double>>>> candidate_path_ptr, 
             std::shared_ptr<std::vector<std::vector<double>>> president_path_ptr,
             std::shared_ptr<sensor_msgs::PointCloud> candidate_msg_ptr, std::shared_ptr<sensor_msgs::PointCloud> president_msg_ptr);

auto obstacle_ptr = std::make_shared<std::vector<std::vector<double>>>();
auto global_path_ptr = std::make_shared<std::vector<std::vector<double>>>();
auto president_path_ptr = std::make_shared<std::vector<std::vector<double>>>();
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
int best_idx = int(num_of_path*0.5) + 1;

int main(int argc, char **argv){
    ros::init(argc, argv, "dwa");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);
    ros::Publisher candidate_pub = nh.advertise<sensor_msgs::PointCloud>("candidate_path", 100);
    ros::Publisher president_pub = nh.advertise<sensor_msgs::PointCloud>("president_path", 100);
    ros::Subscriber obstacle_sub = nh.subscribe("obstacle", 100, obstacle_callback);


    auto candidate_path_ptr = std::make_shared<std::vector<std::vector<std::vector<double>>>>();

    make_global_path();
    make_candidate_path(candidate_path_ptr);

    while(ros::ok()){
        vote_president_path(candidate_path_ptr);
        auto candidate_msg_ptr = std::make_shared<sensor_msgs::PointCloud>();
        auto president_msg_ptr = std::make_shared<sensor_msgs::PointCloud>();
        make_msg(candidate_path_ptr, president_path_ptr, candidate_msg_ptr, president_msg_ptr);
        president_pub.publish(*president_msg_ptr);
        candidate_pub.publish(*candidate_msg_ptr);
        std::cout<<"================"<<std::endl;
        ros::spinOnce();
        loop_rate.sleep();
    }

    
}

void obstacle_callback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    pcl::fromROSMsg(*msg, *cloud_ptr);
}

void make_global_path(){
    auto global_path_len = 5.0;
    global_path_ptr->clear();
    for(int i =0; i < global_path_len * 10; i++){
        std::vector<double> point(3);
        point.at(0) = i * 0.1;
        point.at(1) = 0.0;
        point.at(2) = 0.0;
        global_path_ptr->push_back(point);
    }
}

void make_candidate_path(std::shared_ptr<std::vector<std::vector<std::vector<double>>>> candidate_path_ptr){
    auto av_gap = 2 * angular_velocity / (num_of_path - 1);

    for(int i = 0; i < num_of_path; i++){
        std::vector<std::vector<double>> candidate_i;
        auto current_angular_velocity = angular_velocity - av_gap * i;
        for(int j = 0; j < predict_time * 10; j++){
            std::vector<double> candidate_element = {velocity * j * 0.1 * std::cos(current_angular_velocity * j * 0.1), velocity * j * 0.1 * std::sin(current_angular_velocity * j * 0.1), 0.0};
            candidate_i.push_back(candidate_element); 
        }
        candidate_path_ptr->push_back(candidate_i); 
    }
}

void vote_president_path(std::shared_ptr<std::vector<std::vector<std::vector<double>>>> candidate_path_ptr){
    auto best_cost = 999999.9;
    obstacle_ptr->clear();
    obstacle_ptr->resize(cloud_ptr->size());

    for(int i =0; i < num_of_path; i++){
        double current_cost = 0.0;
        auto current_candidate = candidate_path_ptr->at(i);
        for(int j = 0; j < predict_time * 10; j++){
            // obstacle cost
            for( int k = 0 ; k < cloud_ptr->size(); k++){
                obstacle_ptr->at(k).resize(3);
                obstacle_ptr->at(k).at(0) = cloud_ptr->at(k).x + cloud_ptr->at(k).z * j * 0.1;
                obstacle_ptr->at(k).at(1) = cloud_ptr->at(k).y + cloud_ptr->at(k).intensity * j * 0.1;
                obstacle_ptr->at(k).at(2) = 0.0;

                auto distance = calculateDistance(current_candidate.at(j), obstacle_ptr->at(k));
                if(distance < 0.5){
                    current_cost += 9999;
                }
                // current_cost += obstacle_cost / std::pow(distance, 2);
            }
            // current_cost = current_cost / obstacle_ptr->size();

            // path cost
            double distance;
            if (global_path_ptr->size() >= j){
                distance = calculateDistance(current_candidate.at(j), global_path_ptr->at(j));
            }
            else{
                distance = calculateDistance(current_candidate.at(j), global_path_ptr->at(-1));
            }
            current_cost += global_path_cost * std::pow(distance, 1);
        }
        std::cout<<i<<" cost: "<<current_cost<<std::endl;
        if(current_cost < best_cost){
            best_idx = i;
            best_cost = current_cost;
        }
    }
    president_path_ptr->clear();
    for(int j = 0; j < predict_time * 10; j++){
        president_path_ptr->push_back(candidate_path_ptr->at(best_idx).at(j));
    }

}

void make_msg(std::shared_ptr<std::vector<std::vector<std::vector<double>>>> candidate_path_ptr, 
             std::shared_ptr<std::vector<std::vector<double>>> president_path_ptr,
             std::shared_ptr<sensor_msgs::PointCloud> candidate_msg_ptr, std::shared_ptr<sensor_msgs::PointCloud> president_msg_ptr){

                for(int i = 0; i < candidate_path_ptr->size(); i++){
                    auto candidate_i = candidate_path_ptr->at(i);
                    for(int j = 0; j < candidate_i.size(); j++){
                        auto element = candidate_i.at(j);
                        geometry_msgs::Point32 point;
                        point.x = element.at(0);
                        point.y = element.at(1);
                        point.z = 0.0;
                        candidate_msg_ptr->points.push_back(point);
                    }
                }
                candidate_msg_ptr->header.frame_id = "map";

                for(int j = 0; j < president_path_ptr->size(); j++){
                    auto element = president_path_ptr->at(j);
                    geometry_msgs::Point32 point;
                    point.x = element.at(0);
                    point.y = element.at(1);
                    point.z = 0.0;
                    president_msg_ptr->points.push_back(point);
                }
                president_msg_ptr->header.frame_id = "map";
            }

double calculateDistance(const std::vector<double>& point1, const std::vector<double>& point2) {
    double distance = std::sqrt(
        std::pow(point2[0] - point1[0], 2) +
        std::pow(point2[1] - point1[1], 2) +
        std::pow(point2[2] - point1[2], 2)
    );

    return distance;
}