#include "Morai_Woowa/calibration.h"

calibration::calibration()
{
    ros::NodeHandle nh;
    image_sub = nh.subscribe("python_image", 1, &calibration::image_callBack, this);
    lidar_sub = nh.subscribe("lidar_pre", 1, &calibration::lidar_callBack, this);
    object_sub = nh.subscribe("detected_object", 1, &calibration::object_callBack, this);
                 
    this->do_cali();
}

void calibration::image_callBack(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        // bgr data가 수신되어 그냥 이용해도 된다
        frame = cv::Mat(msg->height, msg->width, CV_8UC3, const_cast<unsigned char*>(msg->data.data()), msg->step);        
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
    if (msg->name == "person")
    {
    ymax = msg->ymax;
    ymin = msg->ymin;
    xmax = msg->xmax;
    xmin = msg->xmin;
    this->projection(frame);
    }

    try{
        cv::imshow("Projection IMG", frame);
        if(cv::waitKey(10) == 27) exit(1);
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("object_callBack %s", e.what());
    }
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
    // // camera 내부 파라미터 값 저장
    // intrinsic << fx, skew_c, cx,
    //              0, fy, cy,
    //              0, 0, 1;

    
    // // LiDAR의 회전을 카메라 좌표계로 변환
    // Eigen::Matrix3d total_rotation = computeRotationMatrix(90, 0, 90);
    
    // // 원점 좌표 저장
    // camera_origin << camera_x, camera_y, camera_z;
    // lidar_origin << lidar_x, lidar_y, lidar_z;

    // // 변환 벡터 계산
    // Eigen::Vector3d translation = lidar_origin - camera_origin;

    // extrinsic.block<3, 3>(0, 0) = total_rotation;   // (0, 0)부터 3x3을 채워넣겠다
    // extrinsic.block<3, 1>(0, 3) = translation;  // (0, 3)부터 3x1을 채워넣겠다


    // std::cout << "Intrinsic parameter: \n";
    // for(int i=0; i < intrinsic.rows(); i++)
    // {
    //     for(int j=0; j < intrinsic.cols(); j++)
    //     {
    //         std::cout << intrinsic(i, j) << " ";
    //     }
    //     std::cout << "\n";
    // }

    // std::cout << "Extrinsic parameter: \n";
    // for(int i=0; i < extrinsic.rows(); i++)
    // {
    //     for(int j=0; j < extrinsic.cols(); j++)
    //     {
    //         std::cout << extrinsic(i, j) << " ";
    //     }
    //     std::cout << "\n";
    // }

        // 카메라 매트릭스, 회전 벡터, 변환 벡터 정의
    cameraMatrix = (cv::Mat_<double>(3, 3) << fx, 0, cx,
                                            0,fy, cy,
                                            0, 0, 1);
    
    // cv::Mat rotate = (cv::Mat_<double>(3, 1) <<  90 * M_PI / 180, 0 * M_PI / 180, 0 * M_PI / 180); // 회전 없음
    // cv::Rodrigues(rotate, rvec);
    // rvec = cv::Mat(3, 3, CV_64F, extrinsic.data());
    rvec = (cv::Mat_<double>(3, 3) << 0, 0, 1,
                                    0,-1, 0,
                                    -1, 0, 0);

                                    
    cv::Mat r_yaw = (cv::Mat_<double>(3, 3) << 0, -1, 0,
                                    1,0, 0,
                                    0, 0, 1);

    rvec = r_yaw * rvec;


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
        // 이미지 포인트를 저장할 벡터
        std::vector<cv::Point2f> imagePoints;

        // 3D 포인트를 2D 이미지 평면으로 투영
        try {
            cv::projectPoints(lidar_points, rvec, tvec, cameraMatrix, distCoeffs, imagePoints);
        } catch (const cv::Exception& e) {
            ROS_ERROR("Projection ERROR! %s", e.what());
        }

        std::unordered_map<int, std::vector<double>> classDistances; // 클래스별 거리 저장
        std::unordered_map<int, int> classCount; // 클래스별 점 수 저장

        for (size_t i=0; i < imagePoints.size(); i++)
        {
            const auto& imagePoint = imagePoints[i];
            int x = static_cast<int>(imagePoint.x); // X 좌표
            int y = static_cast<int>(imagePoint.y); // Y 좌표
            // 박스 안에 있을 때만 점 찍기
            if ((xmin <= x) && (x <= xmax) && (ymin <= y) && (y <= ymax))
            {
                // 점 찍기 (빨간색)
                cv::circle(frame, cv::Point(x, y), 2, cv::Scalar(0, 0, 255), -1);


                // 해당하는 픽셀 좌표의 LiDAR 거리 계산
                const auto& lidarPoint = lidar_points[i]; // 해당하는 Lidar Point

                double distance = std::sqrt(lidarPoint.x * lidarPoint.x + 
                                            lidarPoint.y * lidarPoint.y +
                                            lidarPoint.z * lidarPoint.z);

                int classId = static_cast<int>(intensity[i]); // intensity를 클래스 ID로 변환
                // 거리 추가
                classDistances[classId].push_back(distance);
                classCount[classId]++;
            }
        }

        // 평균 거리 계산
        for (const auto& entry : classDistances) {
            int classId = entry.first;
            const std::vector<double>& distances = entry.second;

            double sum = 0.0;
            for (double dist : distances) {
                sum += dist;
            }
            double averageDistance = sum / classCount[classId];

            std::cout << "Class " << classId << ": Average Distance = " << averageDistance << std::endl;
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


/*
// 3D 포인트 (LiDAR 좌표계)
    std::vector<cv::Point3f> objectPoints = {
        cv::Point3f(7.6422, 2.0477, -0.18369),
        cv::Point3f(4.3972, 0.44665, -0.41235),
        cv::Point3f(3.931, -0.91476, 0),
        cv::Point3f(3.931, -0.91476, 0),
        cv::Point3f(3.931, -0.91476, 0)
        
    };

    // 2D 포인트 (카메라 이미지 좌표계)
    std::vector<cv::Point2f> imagePoints = {
        cv::Point2f(66, 258),
        cv::Point2f(206, 303),
        cv::Point2f(505, 234),
        cv::Point2f(505, 234),
        cv::Point2f(505, 234)
        
    };

    // 카메라 내적 매트릭스 (fx, fy, cx, cy)
    cv::Mat cameraMatrix(3, 3, CV_64F, intrinsic.data());

    // 왜곡 계수 (예: [k1, k2, p1, p2, k3])
    cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64F); // 왜곡이 없는 경우

    // 외부 파라미터 (회전 벡터와 이동 벡터)
    cv::Mat rvec, tvec;

    // solvePnP를 사용하여 외부 파라미터 계산
    // bool success = cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);

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
*/