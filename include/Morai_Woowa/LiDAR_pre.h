#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>


#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/passthrough.h>    // roi
#include <pcl/filters/statistical_outlier_removal.h> // outlier
#include <pcl/filters/voxel_grid.h>     // voxel
// ransac
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>

#define EPSILON 1
#define MIN_POINTS 3

class LiDAR_pre
{
public:
    LiDAR_pre(); // 생성자

    // 라이다 처리 함수
    void cloud_callBack(const sensor_msgs::PointCloud2& msg);
    
    void pose_callBack(const geometry_msgs::PoseStamped& msg);

    // overload
    // Pointer를 Sensor msgs로 바꾸는 함수
    void Pub2Sensor(pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pointer);
    // PC2를 Sensor msgs로 바꾸는 함수
    void Pub2Sensor(pcl::PointCloud<pcl::PointXYZI> pc2);

    // Pointer를 Sensor msgs로 바꾸는 함수
    void Pub2Sensor_utm(pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pointer);
    // PC2를 Sensor msgs로 바꾸는 함수
    void Pub2Sensor_utm(pcl::PointCloud<pcl::PointXYZI> pc2);
    
    // roi 영역 설정
    void roi();
    void outlier();
    void voxel();
    void ransac();
    void dbscan();
    void coord_transform();
    double euclideanDistance(const pcl::PointXYZI& point1, const pcl::PointXYZI& point2);
    void dbscan(double epsilon, int minPoints);

    
private:
    // const sensor_msgs::PointCloud2ConstPtr& input;
    ros::Subscriber point_sub_;
    ros::Subscriber pose_sub_;

    ros::Publisher pub;
    ros::Publisher pub_utm_pcl;

    pcl::PCLPointCloud2 pcl_pc;
    pcl::PointCloud<pcl::PointXYZI> cloud_data; // pointCloud를 받아서 PointXYZI로 변환
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud; 

    geometry_msgs::PoseStamped current_pose;
};