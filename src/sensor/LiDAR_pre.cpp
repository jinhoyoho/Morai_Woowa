#include "Morai_Woowa/LiDAR_pre.h"


// 생성자
LiDAR_pre::LiDAR_pre()
{
    ros::NodeHandle nh;
    point_sub_ = nh.subscribe("/velodyne_points", 1, &LiDAR_pre::cloud_callBack, this);
    pose_sub_ = nh.subscribe("/current_pose", 1, &LiDAR_pre::pose_callBack, this);

    pub = nh.advertise<sensor_msgs::PointCloud2> ("lidar_pre", 1);
    pub_utm_pcl = nh.advertise<sensor_msgs::PointCloud2> ("lidar_utm", 1);
}

// Pointer -> PointCloud2로 변경하는 함수
void LiDAR_pre::Pub2Sensor(pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pointer)
{
    pcl::PCLPointCloud2 pcl_pc_filtered;

    pcl::toPCLPointCloud2(*pcl_pointer, pcl_pc_filtered);

    sensor_msgs::PointCloud2 output;

    pcl_conversions::fromPCL(pcl_pc_filtered, output);

    output.header.frame_id = "map";
    output.header.stamp = ros::Time::now();

    pub.publish(output);
}

// 포인트 클라우드를 ROS 메시지로 변환하여 발행
void LiDAR_pre::Pub2Sensor(pcl::PointCloud<pcl::PointXYZI> pc2)
{
    pcl::PCLPointCloud2 pcl_pc_filtered;

    pcl::toPCLPointCloud2(pc2, pcl_pc_filtered);

    sensor_msgs::PointCloud2 output;

    pcl_conversions::fromPCL(pcl_pc_filtered, output);

    output.header.frame_id = "map";
    output.header.stamp = ros::Time::now();

    pub.publish(output);
}

void LiDAR_pre::Pub2Sensor_utm(pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pointer)
{
    pcl::PCLPointCloud2 pcl_pc_filtered;

    pcl::toPCLPointCloud2(*pcl_pointer, pcl_pc_filtered);

    sensor_msgs::PointCloud2 output;

    pcl_conversions::fromPCL(pcl_pc_filtered, output);

    output.header.frame_id = "map";
    output.header.stamp = ros::Time::now();

    pub_utm_pcl.publish(output);
}

// 포인트 클라우드를 ROS 메시지로 변환하여 발행
void LiDAR_pre::Pub2Sensor_utm(pcl::PointCloud<pcl::PointXYZI> pc2)
{
    pcl::PCLPointCloud2 pcl_pc_filtered;

    pcl::toPCLPointCloud2(pc2, pcl_pc_filtered);

    sensor_msgs::PointCloud2 output;

    pcl_conversions::fromPCL(pcl_pc_filtered, output);

    output.header.frame_id = "map";
    output.header.stamp = ros::Time::now();

    pub_utm_pcl.publish(output);
}

void LiDAR_pre::cloud_callBack(const sensor_msgs::PointCloud2& msg)
{
   pcl_conversions::toPCL(msg, pcl_pc); // ROS -> PCLPointCloud2
   pcl::fromPCLPointCloud2(pcl_pc, cloud_data); // PCLPointCloud2 -> PointXYZI

   this->roi(); // 관심 영역 설정
   this->voxel(); // Down sampling(voxel) 실행
   this->outlier(); // outlier 제거
   this->ransac(); // ransac 실행

   Pub2Sensor(cloud_data);

   this->coord_transform();

   Pub2Sensor_utm(cloud_data);

}

void LiDAR_pre::pose_callBack(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    global_x_ = static_cast<float>(msg->x);
    global_y_ = static_cast<float>(msg->y);
    global_yaw_ = static_cast<float>(msg->theta);
}

void LiDAR_pre::roi()
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr raw_data_p_ = cloud_data.makeShared();

    pcl::PassThrough<pcl::PointXYZI> pass;

    // Apply Passthrough Filter
    pass.setInputCloud(raw_data_p_);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-2, 1);   // 상하거리
    pass.filter(*raw_data_p_);

    pass.setInputCloud(raw_data_p_);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(0, 55);  // 앞뒤거리
    pass.filter(*raw_data_p_);

    pass.setInputCloud(raw_data_p_);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-5.0, 5.0); //좌우거리
    pass.filter(*raw_data_p_);

    cloud_data = *raw_data_p_; // 필터링된 데이터를 cloud_data에 저장
}

void LiDAR_pre::voxel()
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr roi_data_p_ = cloud_data.makeShared();

    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud(roi_data_p_);              //입력
    sor.setLeafSize(0.1f, 0.1f, 0.1f); //leaf size  1cm 
    sor.filter(*roi_data_p_);          //출력 

    cloud_data = *roi_data_p_; // Voxel 데이터를 cloud_data에 저장
}

void LiDAR_pre::outlier()
{

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr outlier_p_ = cloud_data.makeShared();

    // 1. Statistical Outlier Removal
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud(outlier_p_);            //입력 
    sor.setMeanK(100);                    //분석시 고려한 이웃 점 수
    sor.setStddevMulThresh(0.1);         //Outlier로 처리할 거리 정보 
    sor.filter(*cloud_filtered);         // 필터 적용

    cloud_data = *cloud_filtered;
}


void LiDAR_pre::ransac()
{
    // 평면 포인트와 비평면 포인트를 저장하기 위한 PointCloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr inlierPoints (new pcl::PointCloud<pcl::PointXYZI>),
										inlierPoints_neg (new pcl::PointCloud<pcl::PointXYZI>);

    pcl::PointCloud<pcl::PointXYZI>::Ptr voxel_data_p_ = cloud_data.makeShared();   // Pointer

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients()); // 평면 계수를 저장하는 포인터
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());    // 평면 모델에 적합한 포인트 인덱스를 저장하기 위한 포인터


	// 오프젝트 생성 Create the segmentation object.
	pcl::SACSegmentation<pcl::PointXYZI> seg;   // SAC(Sample Consensus) 세그멘테이션 객체 생성
	seg.setOptimizeCoefficients(true);        // 모델 계수의 정제를 활성화
	seg.setModelType(pcl::SACMODEL_PLANE);    // 모델 타입은 평면
	seg.setMethodType(pcl::SAC_RANSAC);       // RANSAC 방법 사용
	seg.setMaxIterations(3000);               // 최대 실행 수
	seg.setDistanceThreshold(0.08);          // 최대 거리
	seg.setInputCloud(voxel_data_p_);        //입력 클라우드
	seg.segment(*inliers, *coefficients);    // 인라이어 인덱스와 모델 계수를 계산

	pcl::copyPointCloud<pcl::PointXYZI>(*voxel_data_p_, *inliers, *inlierPoints);

	pcl::ExtractIndices<pcl::PointXYZI> extract;
	extract.setInputCloud(voxel_data_p_);
	extract.setIndices(inliers);
	extract.setNegative(true); //false
	extract.filter(*inlierPoints_neg);

    cloud_data = *inlierPoints_neg;
}

void LiDAR_pre::coord_transform()
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr raw_data_p_ = cloud_data.makeShared();

    // Transform point cloud to global frame using localization data
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << global_x_, global_y_, 0.0;
    transform.rotate(Eigen::AngleAxisf(global_yaw_, Eigen::Vector3f::UnitZ()));

    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::transformPointCloud(*raw_data_p_, *transformed_cloud, transform);
    
    cloud_data = *transformed_cloud; 
}


// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "LiDAR_preprocessing");

//     LiDAR_pre lp;

//     ros::spin();
//     return 0;
// }