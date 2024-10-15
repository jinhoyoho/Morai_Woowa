#include "Morai_Woowa/calibration.h"

calibration::calibration(ros::NodeHandle& nh):PCAserver_(nh, "/person_collision_action", boost::bind(&calibration::execute, this, _1) ,false)
{
    PCAserver_.start(); // 액션서버 시작
    lidar_sub = nh.subscribe("lidar_pre", 1, &calibration::lidar_callBack, this);
    object_sub = nh.subscribe("person", 1, &calibration::object_callBack, this);
    gps_sub = nh.subscribe("gps", 1, &calibration::gps_callBack, this);
    imu_sub = nh.subscribe("imu", 1, &calibration::imu_callBack, this);
    min_distance = 987654321.0;    // 최소 거리 갱신

    heading = 0;    // dilly의 헤딩

    this->do_cali();
}

void calibration::lidar_callBack(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    last_lidar_time = msg->header.stamp; // 라이다 스탬프

    pcl::PCLPointCloud2 cloud_intermediate; // 포인트 클라우드 데이터 저장

    // 포인터가 가리키는 포인트 클라우드 데이터를 cloud_intermediate에 저장
    pcl_conversions::toPCL(*msg, cloud_intermediate);
    // cloud_intermediate에 저장된 PCL 포인트 클라우드 데이터를 cloud 객체로 변환
    pcl::fromPCLPointCloud2(cloud_intermediate, cloud);

    // std::vector<cv::Point3f>로 변환
    std::vector<cv::Point3f> objectPoints;

    // 포인트 변환
    for (size_t i = 0; i < cloud.size(); ++i) {
        objectPoints.emplace_back(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
        intensity.emplace_back(cloud.points[i].intensity);
    }
    lidar_points = objectPoints;
}


void calibration::object_callBack(const morai_woowa::obj_info::ConstPtr& msg)
{
    last_image_time = msg->image.header.stamp; // 이미지 스탬프

    if ((last_image_time - last_lidar_time).toSec() < 0.01) { // 10ms 이내의 차이
            
        frame = cv::Mat(msg->image.height, msg->image.width, CV_8UC3, const_cast<unsigned char*>(msg->image.data.data()), msg->image.step);

        if (msg->name == "person")
        {
            ymax = msg->ymax;
            ymin = msg->ymin;
            xmax = msg->xmax;
            xmin = msg->xmin;
            this->projection(frame);
        }
        cv::imshow("Projection Image", frame);
        if(cv::waitKey(10) == 27) exit(-1);
    }
}

void calibration::execute(const morai_woowa::Person_Collision_ActGoalConstPtr& goal)
{
    ROS_INFO("Execute action: Person Collision Action!");

    // 찾는 범위 보다 크다면 false (987654321 포함) -> 오작동 같은 예외처리 필요!
    if(goal->range < min_distance)
    {   
        ROS_INFO("Action Fail: Do not meet range");
        // 모두 실패!
        feedback_.is_target = false;
        PCAserver_.publishFeedback(feedback_);
    }
    // else if(min_distance < 0)
}

void calibration::gps_callBack(const morai_msgs::GPSMessage::ConstPtr& msg)
{
    last_gps_time = msg->header.stamp;  // gps 시간 측정
    if ((last_imu_time - last_gps_time).toSec() < 0.01)
    {
        double latitude = msg.latitude
        double longitude = msg.longitude
        double x_offset = msg.eastOffset
        double y_offset = msg.northOffset
    }
}

void calibration::imu_callBack(const sensor_msgs::Imu::ConstPtr& msg)
{
    last_imu_time = msg->header.stamp;  // imu 시간 측정
    // 쿼터니언 가져오기
    geometry_msgs::Quaternion orientation = msg->orientation;

    // 쿼터니언에서 yaw 각도 계산
    double yaw = atan2(2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z));

    // 라디안을 도로 변환
    heading = yaw * (180.0 / M_PI);

    ROS_INFO("Heading (Yaw): %f degrees", heading);
}

cv::Mat_<double> calibration::computeRotationMatrix(double roll, double pitch, double yaw) {
    // 각도를 라디안으로 변환
    roll *= M_PI / 180.0;
    pitch *= M_PI / 180.0;
    yaw *= M_PI / 180.0;

    // 회전 행렬 계산
    cv::Mat_<double> R_x(3, 3);
    R_x << 1, 0, 0,
           0, cos(roll), -sin(roll),
           0, sin(roll), cos(roll);

    cv::Mat_<double> R_y(3, 3);
    R_y << cos(pitch), 0, sin(pitch),
           0, 1, 0,
           -sin(pitch), 0, cos(pitch);

    cv::Mat_<double> R_z(3, 3);
    R_z << cos(yaw), -sin(yaw), 0,
           sin(yaw), cos(yaw), 0,
           0, 0, 1;

    // 전체 회전 행렬
    return R_z * R_y * R_x;
}


void calibration::do_cali()
{
    // 카메라 내부 매트릭스, 회전 벡터, 변환 벡터 정의
    cameraMatrix = (cv::Mat_<double>(3, 3) << fx, 0, cx,
                                            0,fy, cy,
                                            0, 0, 1);

    // 라이다 -> 카메라 좌표계로 일치시키는 행렬
    rvec = (cv::Mat_<double>(3, 3) << 0, 1, 0,
                                     0, 0, 1,
                                     -1, 0, 0);

    rvec = rvec * this->computeRotationMatrix(0, 0, 0);


    tvec = (cv::Mat_<double>(3, 1) << lidar_x - camera_x, lidar_y - camera_y, lidar_z - camera_z); 
    distCoeffs = cv::Mat::zeros(4, 1, CV_64F); // 왜곡 없음

    std::cout << "cameraMatrix: \n" << cameraMatrix << "\n\n";
    std::cout << "rvec: \n" << rvec << "\n\n";
    std::cout << "tvec: \n" << tvec << "\n\n";
    std::cout << "distCoeffs: \n" << distCoeffs << "\n\n";
}


void calibration::projection(cv::Mat frame)
{
    try{
        // lidar_points가 존재했을때 실행
        if(lidar_points.size())
        {
             // 이미지 포인트를 저장할 벡터
            std::vector<cv::Point2f> imagePoints;
            
            // 3D 포인트를 2D 이미지 평면으로 투영
            cv::projectPoints(lidar_points, rvec, tvec, cameraMatrix, distCoeffs, imagePoints);
            
            std::map<int, std::vector<cv::Point3f>> classPoints;    // 클래스별 라이다 좌표 저장


            for (size_t i=0; i < imagePoints.size(); i++)
            {
                const auto& imagePoint = imagePoints[i];
                int x = static_cast<int>(imagePoint.x); // X 좌표
                int y = static_cast<int>(imagePoint.y); // Y 좌표

                // 박스 안에 있을 때만 점 찍기
                if ((xmin <= x) && (x <= xmax) && (ymin <= y) && (y <= ymax))
                {
                    // 해당하는 픽셀 좌표의 LiDAR
                    const auto& lidarPoint = lidar_points[i]; // 해당하는 Lidar Point

                    int classId = static_cast<int>(intensity[i]); // intensity를 클래스 ID로 변환
                    classPoints[classId].push_back(lidarPoint); // 라이다 좌표 저장
                }
            }
            
            int minClassId = -1;
            min_distance = std::numeric_limits<double>::max(); // 무한대로 초기화
            // 평균 좌표
            double average_x;
            double average_y;
            double average_z;
            
            // 각 클래스에 대해 평균 거리 계산
            for (const auto& pair : classPoints) {
                int classId = pair.first;
                const std::vector<cv::Point3f>& points = pair.second;

                if (points.empty()) {
                    continue; // 포인트가 없는 클래스는 건너뜁니다
                }

                cv::Point3f average = {0.0, 0.0, 0.0};

                // 모든 포인트의 합산
                for (const auto& point : points) {
                    average.x += point.x;
                    average.y += point.y;
                    average.z += point.z;
                }

                // 평균 계산
                average.x /= points.size();
                average.y /= points.size();
                average.z /= points.size();

                // 거리 계산
                double averageDistance = std::sqrt(average.x * average.x + 
                                                    average.y * average.y + 
                                                    average.z * average.z);

                // 가장 작은 평균 거리 업데이트
                if (averageDistance < min_distance) {
                    min_distance = averageDistance;
                    minClassId = classId;
                    average_x = average.x;
                    average_y = average.y;
                    average_z = average.z;
                }
            }

        
            std::vector<cv::Point2f> imageProjections; // 이미지에 투영시킬 좌표
            if (minClassId != -1) // -1이 아닐때 실행
            {
                cv::projectPoints(classPoints[minClassId], rvec, tvec, cameraMatrix, distCoeffs, imageProjections);

                for(size_t i=0; i < imageProjections.size(); i++)
                {
                    const auto& imageProjection = imageProjections[i];
                    int x = static_cast<int>(imageProjection.x);
                    int y = static_cast<int>(imageProjection.y);

                    cv::circle(frame, cv::Point(x, y), 3, cv::Scalar(0, 0, 255), -1); // 점 찍기
                }


                std::cout << "Average Distance: " << min_distance << "\n";
                std::cout << "Average Coord: " << average_x << " " << average_y << " " << average_z << "\n";
            }
        }
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Calibration ERROR! %s", e.what());
    }
}