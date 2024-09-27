#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>    // roi
#include <pcl/filters/voxel_grid.h>     // voxel
// ransac
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>

// outlier
#include <pcl/filters/statistical_outlier_removal.h>

class LiDAR_pre
{
public:
    LiDAR_pre(); // 생성자

    // 라이다 처리 함수
    void cloud_callBack(const sensor_msgs::PointCloud2& msg);

    // overload
    // Pointer를 Sensor msgs로 바꾸는 함수
    void Pub2Sensor( pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pointer);
    // PC2를 Sensor msgs로 바꾸는 함수
    void Pub2Sensor(pcl::PointCloud<pcl::PointXYZI> pc2);
    
    // roi 영역 설정
    void roi();
    void outlier();
    void voxel();
    void ransac();
    void dbscan();
    
private:
    // const sensor_msgs::PointCloud2ConstPtr& input;
    ros::Subscriber point_sub_;
    pcl::PCLPointCloud2 pcl_pc;
    ros::Publisher pub;
    pcl::PointCloud<pcl::PointXYZI> cloud_data; // pointCloud를 받아서 PointXYZI로 변환
};