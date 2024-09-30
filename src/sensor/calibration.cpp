#include "Morai_Woowa/calibration.h"
#include "Morai_Woowa/LiDAR_pre.h"

calibration::calibration()
{
    ros::NodeHandle nh;
    image_sub = nh.subscribe("python_image", 1, &calibration::image_callBack, this);
    lidar_sub = nh.subscribe("lidar_pre", 1, &calibration::lidar_callBack, this);
    
    this->do_cali(); // calibration 실행
    
}

void calibration::image_callBack(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        // bgr data가 수신되어 그냥 이용해도 된다
        frame = cv::Mat(msg->height, msg->width, CV_8UC3, const_cast<unsigned char*>(msg->data.data()), msg->step);

        this->projection(); // 투영

        // 변환된 이미지를 보여줍니다.
        cv::imshow("Received Image", frame);
        if (cv::waitKey(10) == 27) exit(1);  // esc키로 종료  
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert image! %s", e.what());
        ROS_ERROR("Received image encoding: %s", msg->encoding.c_str());
    }
}

void calibration::lidar_callBack(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    pcl::PCLPointCloud2 cloud_intermediate; // 포인트 클라우드 데이터 저장
    pcl::PointCloud<pcl::PointXYZ> cloud;   // pointcloud 저장

    // 포인터가 가리키는 포인트 클라우드 데이터를 cloud_intermediate에 저장
    pcl_conversions::toPCL(*msg, cloud_intermediate);
    // cloud_intermediate에 저장된 PCL 포인트 클라우드 데이터를 cloud 객체로 변환
    pcl::fromPCLPointCloud2(cloud_intermediate, cloud);

    // // std::vector<cv::Point3f>로 변환
    // std::vector<cv::Point3f> objectPoints;

    // // 포인트 변환
    // for (size_t i = 0; i < cloud.size(); ++i) {
    //     objectPoints.emplace_back(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
    // }

    Eigen::MatrixXd points(cloud.size(), 3); // x, y, z

    for (size_t i = 0; i < cloud.size(); i++) {
        points(i, 0) = cloud.points[i].x;       // x
        points(i, 1) = cloud.points[i].y;       // y
        points(i, 2) = cloud.points[i].z;       // z
    }

    // lidar_points에 저장
    lidar_points = points;
}

Eigen::Matrix3d calibration::computeRotationMatrix(double roll, double pitch, double yaw) {
    // 각도를 라디안으로 변환
    roll *= M_PI / 180.0;
    pitch *= M_PI / 180.0;
    yaw *= M_PI / 180.0;

    // 회전 행렬 계산
    Eigen::Matrix3d R_x;
    R_x << 1, 0, 0,
           0, cos(roll), -sin(roll),
           0, sin(roll), cos(roll);

    Eigen::Matrix3d R_y;
    R_y << cos(pitch), 0, sin(pitch),
           0, 1, 0,
           -sin(pitch), 0, cos(pitch);

    Eigen::Matrix3d R_z;
    R_z << cos(yaw), -sin(yaw), 0,
           sin(yaw), cos(yaw), 0,
           0, 0, 1;

    // 전체 회전 행렬
    return R_z * R_y * R_x;
}

void calibration::do_cali()
{
    // // 카메라 매트릭스, 회전 벡터, 변환 벡터 정의
    // cameraMatrix = (cv::Mat_<double>(3, 3) << fx, 0, cx,
    //                                         0,fy, cy,
    //                                         0, 0, 1);
    
    // cv::Mat rotate = (cv::Mat_<double>(3, 1) <<  90 * M_PI / 180, 0 * M_PI / 180, 0 * M_PI / 180); // 회전 없음
    // cv::Rodrigues(rotate, rvec);

    // tvec = (cv::Mat_<double>(3, 1) << lidar_x - camera_x, lidar_y - camera_y, lidar_z - camera_z); 
    // distCoeffs = cv::Mat::zeros(4, 1, CV_64F); // 왜곡 없음

    // std::cout << "cameraMatrix: \n" << cameraMatrix << "\n\n";
    // std::cout << "rvec: \n" << rvec << "\n\n";
    // std::cout << "tvec: \n" << tvec << "\n\n";
    // std::cout << "distCoeffs: \n" << distCoeffs << "\n\n";

    // camera 내부 파라미터 값 저장
    intrinsic << fx, skew_c, cx,
                 0, fy, cy,
                 0, 0, 1;


    // camera 외부 파라미터 값 저장
    // 카메라 회전 행렬 계산
    Eigen::Matrix3d camera_rotation = computeRotationMatrix(camera_roll, camera_pitch, camera_yaw);
    
    // 원점 좌표 저장
    camera_origin << camera_x, camera_y, camera_z;
    lidar_origin << lidar_x, lidar_y, lidar_z;

    // 변환 벡터 계산
    Eigen::Vector3d translation = camera_origin - lidar_origin;

    extrinsic.block<3, 3>(0, 0) = camera_rotation;   // (0, 0)부터 3x3을 채워넣겠다
    extrinsic.block<3, 1>(0, 3) = translation;  // (0, 3)부터 3x1을 채워넣겠다

    std::cout << "Intrinsic Matrix: \n";

    for (int i=0; i < intrinsic.rows(); i++)
    {
        for(int j=0; j < intrinsic.cols(); j++)
        {
            std::cout << intrinsic(i, j) << " ";
        }
        std::cout << "\n";
    }

    std::cout << "Extrinsic Matrix: \n";

    for (int i=0; i < extrinsic.rows(); i++)
    {
        for(int j=0; j < extrinsic.cols(); j++)
        {
            std::cout << extrinsic(i, j) << " ";
        }
        std::cout << "\n";
    }
}


// 라이다 점을 이미지에 투영
void calibration::projection()
{
    // // 이미지 포인트를 저장할 벡터
    // std::vector<cv::Point2f> imagePoints;

    // // 3D 포인트를 2D 이미지 평면으로 투영
    // try {
    //     cv::projectPoints(lidar_points, rvec, tvec, cameraMatrix, distCoeffs, imagePoints);
    // } catch (const cv::Exception& e) {
    //     std::cerr << "OpenCV Exception: " << e.what() << std::endl;
    //     return; // 오류 처리
    // }

    // for (const auto& imagePoint : imagePoints)
    // {
    //     std::cout << imagePoint.x << " " << imagePoint.y << "\n";
    //     cv::circle(frame, imagePoint, 3, cv::Scalar(255, 0, 255), -1);
    // }

    // 결과를 저장할 벡터
    std::vector<Eigen::Vector3d> results; // 3D 포인트를 저장할 벡터

    for(int i=0; i < lidar_points.cols(); i++)
    {
        Eigen::Vector4d point;
        point << lidar_points(0, i), lidar_points(1, i), lidar_points(2, i), 1; // 동차 좌표로 변환

        // 결과 저장
        Eigen::Vector3d result = intrinsic * extrinsic * point; // 3x1 벡터
        results.push_back(result); // 벡터에 추가
    }

    // OpenCV에서 점을 찍기
    for (const auto& result : results) {
        int x = static_cast<int>(result[0]); // X 좌표
        int y = static_cast<int>(result[1]); // Y 좌표

        // std::cout << x << " " << y << "\n";

        // 점 찍기 (빨간색, 두께 5)
        cv::circle(frame, cv::Point(x, y), 5, cv::Scalar(0, 0, 255), -1);
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "calibration");

    ROS_INFO("Start Calibration...");

    calibration cl;
    LiDAR_pre lp;

    ros::spin();

    return 0;
}