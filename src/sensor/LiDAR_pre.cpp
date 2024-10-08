#include "Morai_Woowa/LiDAR_pre.h"
//#include "LiDAR_pre.h"


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
   this->dbscan(EPSILON, MIN_POINTS);

   Pub2Sensor(cloud_data);

   this->coord_transform();

   Pub2Sensor_utm(transformed_cloud);
}

void LiDAR_pre::pose_callBack(const geometry_msgs::PoseStamped &msg)
{
    current_pose = msg;
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

// 포인트 간의 유클리드 거리 계산 함수
double LiDAR_pre::euclideanDistance(const pcl::PointXYZI& point1, const pcl::PointXYZI& point2) {
    return std::sqrt(std::pow(point1.x - point2.x, 2) +
                     std::pow(point1.y - point2.y, 2) +
                     std::pow(point1.z - point2.z, 2));
}

// DBSCAN 알고리즘
void LiDAR_pre::dbscan(double epsilon, int minPoints) {
    int numPoints = cloud_data.size();
    std::vector<bool> visited(numPoints, false);    // 방문 여부 체크
    std::vector<int> cluster(numPoints, -1);        // 클러스터 번호, -1은 노이즈
    int clusterID = 0;

    for (int i = 0; i < numPoints; ++i) {
        if (visited[i]) continue;

        visited[i] = true;
        std::vector<int> neighbors;

        // 이웃 포인트 찾기
        for (int j = 0; j < numPoints; ++j) {
            if (euclideanDistance(cloud_data.points[i], cloud_data.points[j]) <= epsilon) {
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
                        if (euclideanDistance(cloud_data.points[neighborIndex], cloud_data.points[j]) <= epsilon) {
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

    for(int i=0; i<cluster.size(); i++){
        cloud_data.points[i].intensity = cluster.at(i);
    }
}

void LiDAR_pre::coord_transform()
{
    transformed_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr data_p_ = cloud_data.makeShared(); // Pointer

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
    for (const auto& point : data_p_->points) {
        pcl::PointXYZI transformed_point;

        // 회전 변환
        transformed_point.x = transform(0, 0) * point.x + transform(0, 1) * point.y + transform(0, 2) * point.z + current_pose.pose.position.x;//transform(0, 3);
        transformed_point.y = transform(1, 0) * point.x + transform(1, 1) * point.y + transform(1, 2) * point.z + current_pose.pose.position.y;//transform(1, 3);
        transformed_point.z = transform(2, 0) * point.x + transform(2, 1) * point.y + transform(2, 2) * point.z + current_pose.pose.position.z;//transform(2, 3);
        transformed_point.intensity = point.intensity; // 강도 정보 유지

        // 변환된 포인트를 클라우드에 추가
        transformed_cloud->points.push_back(transformed_point);
    }

    transformed_cloud->width = transformed_cloud->points.size();
    transformed_cloud->height = 1; // 비정형 포인트 클라우드
    transformed_cloud->is_dense = false; // NaN 포인트가 있을 수 있으므로

    // 변환된 포인트 클라우드 확인
    std::cout << std::fixed << std::setprecision(10);

    for (const auto& point : transformed_cloud->points) {
        std::cout << "Transformed Point: (" << point.x << ", " << point.y << ", " << point.z << ")\n";
    }
}

// void LiDAR_pre::coord_transform()
// {

//     transformed_cloud.reset(new pcl::PointCloud<pcl::PointfD>());
//     pcl::PointCloud<pcl::PointXYZI>::Ptr data_p_ = cloud_data.makeShared();   // Pointer
    
//     Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();

//     tf::Quaternion q(
//         current_pose.pose.orientation.x,
//         current_pose.pose.orientation.y,
//         current_pose.pose.orientation.z,
//         current_pose.pose.orientation.w
//     );

//     tf::Matrix3x3 rotation_matrix(q);

//     for(int i = 0; i<3; i++){
//         for(int j = 0; j<3; j++){
//             transform(i,j) = rotation_matrix[i][j];
//         }
//     }
//     // Ensure values are converted to double
//     transform(0, 3) = static_cast<double>(current_pose.pose.position.x);
//     transform(1, 3) = static_cast<double>(current_pose.pose.position.y);
//     transform(2, 3) = static_cast<double>(current_pose.pose.position.z);

// // std::cout << std::fixed << std::setprecision(10); // 소수점 이하 10자리까지 출력
// // std::cout << transform(0, 3) << " " << transform(1, 3) << std::endl;

//     pcl::transformPointCloud(*data_p_, *transformed_cloud, transform);

// }

// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "LiDAR_preprocessing");

//     LiDAR_pre lp;

//     ros::spin();
//     return 0;
// }