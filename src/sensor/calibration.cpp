#include "Morai_Woowa/calibration.h"
#include "Morai_Woowa/LiDAR_pre.h"

calibration::calibration()
{
    ros::NodeHandle nh;
    image_sub = nh.subscribe("python_image", 1, &calibration::image_callBack, this);
    lidar_sub = nh.subscribe("lidar_pre", 1, &calibration::lidar_callBack, this);
    
    intrinsic << fx, skew_c, cx,
                 0, fy, cy,
                 0, 0, 1;
                 
    this->do_cali();
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


void calibration::do_cali()
{
    // 3D 포인트 (LiDAR 좌표계)
    std::vector<cv::Point3f> objectPoints = {
        cv::Point3f(3.8865, 2.8769, 0.93993),
        cv::Point3f(6.0686, 0.057146, 0.74517),
        cv::Point3f(3.7771, -2.4609, 0.87629),
        cv::Point3f(3.8467, 2.8538, 0.25102),
        cv::Point3f(6.044, 0.051695, 0.1055),
        cv::Point3f(3.7597, -2.4438, 0.23501),
        cv::Point3f(3.8965, 2.7406, -0.41678),
        cv::Point3f(6.0707, -0.10597, -0.3182),
        cv::Point3f(3.7982, -2.6378, -0.40458)
    };

    // 2D 포인트 (카메라 이미지 좌표계)
    std::vector<cv::Point2f> imagePoints = {
        cv::Point2f(88, 184),
        cv::Point2f(319, 201),
        cv::Point2f(523, 178),
        cv::Point2f(80, 240),
        cv::Point2f(317, 234),
        cv::Point2f(527, 229),
        cv::Point2f(100, 278),
        cv::Point2f(325, 266),
        cv::Point2f(542, 275)
    };

    // 카메라 내적 매트릭스 (fx, fy, cx, cy)
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 
        fx, skew_c, cx,
        0, fy, cy,
        0, 0, 1);

    // 왜곡 계수 (예: [k1, k2, p1, p2, k3])
    cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64F); // 왜곡이 없는 경우

    // 외부 파라미터 (회전 벡터와 이동 벡터)
    cv::Mat rvec, tvec;

    // solvePnP를 사용하여 외부 파라미터 계산
    bool success = cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);

    if (success) {
        std::cout << "Rotation Vector:\n" << rvec << std::endl;
        std::cout << "Translation Vector:\n" << tvec << std::endl;

        // 회전 벡터를 회전 행렬로 변환
        cv::Mat rotationMatrix;
        cv::Rodrigues(rvec, rotationMatrix);
        std::cout << "Rotation Matrix:\n" << rotationMatrix << std::endl;

        // Eigen으로 변환
        Eigen::Matrix3d R; // 3x3 회전 행렬
       

        // OpenCV 회전 행렬을 Eigen으로 변환
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                R(i, j) = rotationMatrix.at<double>(i, j);
            }
        }

        // 이동 벡터를 Eigen으로 변환
        Eigen::Vector3d t;
        t << tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2);

        // extrinsic 매트릭스 구성
        extrinsic.block<3, 3>(0, 0) = R; // 회전 행렬
        extrinsic.block<3, 1>(0, 3) = t; // 이동 벡터

        // 결과 출력
        std::cout << "Extrinsic Parameters:\n" << extrinsic << std::endl;
    } else {
        std::cout << "solvePnP failed!" << std::endl;
    }
}



void calibration::projection()
{
    // LiDAR 포인트의 shape 출력
    std::cout << "Shape of lidar_points: " << lidar_points.rows() << " x " << lidar_points.cols() << std::endl;

    // 결과를 저장할 벡터
    std::vector<Eigen::Vector3d> results; // 3D 포인트를 저장할 벡터

    for(int i = 0; i < lidar_points.rows(); i++)
    {
        Eigen::Vector4d point;
        point << lidar_points(i, 0), lidar_points(i, 1), lidar_points(i, 2), 1; // 동차 좌표로 변환
        std::cout << "LiDAR Point: " << point.transpose() << "\n"; // LiDAR 포인트 출력

        // 결과 저장
        Eigen::Vector3d result = intrinsic * extrinsic * point; // 3x1 벡터
        results.push_back(result); // 벡터에 추가
    }

    // OpenCV에서 점을 찍기
    for (const auto& result : results) {
        int x = static_cast<int>(result[0] / result[2]); // X 좌표
        int y = static_cast<int>(result[1] / result[2]); // Y 좌표

        // std::cout << x << " " << y << "\n";

        // 점 찍기 (빨간색, 두께 5)
        cv::circle(frame, cv::Point(x, y), 1, cv::Scalar(0, 0, 255), -1);
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