#include "Morai_Woowa/calibration.h"

calibration::calibration()
{
    ros::NodeHandle nh;
    lidar_sub = nh.subscribe("lidar_pre", 1, &calibration::lidar_callBack, this);
    object_sub = nh.subscribe("person", 1, &calibration::object_callBack, this);
    double min_x = 0;   // 최소 좌표
    double min_y = 0;
    double min_z = 0;

    this->do_cali();
}

void calibration::lidar_callBack(const sensor_msgs::PointCloud2ConstPtr& msg)
{
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

    rvec = this->computeRotationMatrix(-10, 0, 0) * rvec;


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
        if(lidar_points.size())
        {
             // 이미지 포인트를 저장할 벡터
            std::vector<cv::Point2f> imagePoints;
            
            // 3D 포인트를 2D 이미지 평면으로 투영
            cv::projectPoints(lidar_points, rvec, tvec, cameraMatrix, distCoeffs, imagePoints);
            
            std::unordered_map<int, std::vector<double>> classDistances; // 클래스별 거리 저장
            std::unordered_map<int, int> classCount; // 클래스별 점 수 저장
            std::map<int, std::vector<cv::Point3f>> classPoints;    // 클래스별 라이다 좌표 저장


            for (size_t i=0; i < imagePoints.size(); i++)
            {
                const auto& imagePoint = imagePoints[i];
                int x = static_cast<int>(imagePoint.x); // X 좌표
                int y = static_cast<int>(imagePoint.y); // Y 좌표

                // 박스 안에 있을 때만 점 찍기
                if ((xmin <= x) && (x <= xmax) && (ymin <= y) && (y <= ymax))
                {
                    // 점 찍기 (빨간색)
                    cv::circle(frame, cv::Point(x, y), 3, cv::Scalar(0, 0, 255), -1);


                    // 해당하는 픽셀 좌표의 LiDAR 거리 계산
                    const auto& lidarPoint = lidar_points[i]; // 해당하는 Lidar Point

                    double distance = std::sqrt(lidarPoint.x * lidarPoint.x + 
                                                lidarPoint.y * lidarPoint.y +
                                                lidarPoint.z * lidarPoint.z);

                    int classId = static_cast<int>(intensity[i]); // intensity를 클래스 ID로 변환
                    classDistances[classId].push_back(distance);    // 거리 저장
                    classCount[classId]++;  // 개수 증가
                    classPoints[classId].push_back(lidarPoint); // 라이다 좌표 저장
                }
            }

            
            std::map<int, std::vector<cv::Point3f>> averagePoints;    // 클래스별 라이다 좌표 저장

            // 클래스별 평균 계산 및 출력
            for (const auto& entry : classPoints) {
                int classId = entry.first;
                const std::vector<cv::Point3f>& points = entry.second;

                cv::Point3f average = {0.0, 0.0, 0.0};
                for (const auto& point : points) {
                    average.x += point.x;
                    average.y += point.y;
                    average.z += point.z;
                }

                if (!points.empty()) {
                    average.x /= points.size();
                    average.y /= points.size();
                    average.z /= points.size();
                }
                averagePoints[classId].push_back(average);  // 평균 좌표 저장
            }

            double min_distance = 987654321.0;    // 최소 거리
            

            // 평균 거리 계산
            for (const auto& entry : classDistances) {
                int classId = entry.first;
                const std::vector<double>& distances = entry.second;

                double sum = 0.0;
                for (double dist : distances) {
                    sum += dist;
                }

                double averageDistance = sum / classCount[classId];

                if (min_distance > averageDistance) // 거리가 더 작다면
                {
                    min_distance = averageDistance; // 거리 갱신

                    const std::vector<cv::Point3f>& averagePointPointer = averagePoints[classId];
                    if(!averagePointPointer.empty())
                    {
                        min_x = averagePointPointer[0].x;   // 최소 거리 라이다 x 좌표
                        min_y = averagePointPointer[0].y;   // 최소 거리 라이다 y 좌표
                        min_z = averagePointPointer[0].z;   // 최소 거리 라이다 z 좌표
                    }
                }
            }

            // min_distance가 987654321 이 아닐때
            if (min_distance != 987654321.0)
            {
                std::cout << "Distance: " << min_distance << "\n";
                std::cout << "최소 라이다 좌표: " << min_x << " " << min_y << " " << min_z << "\n";
            }
        }
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Calibration ERROR! %s", e.what());
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "calibration");

    ROS_INFO("Start Calibration, LiDAR_pre, Traffic...");
    LiDAR_pre lp;

    calibration cl;
    Traffic tf;

    ros::spin();

    return 0;
}

