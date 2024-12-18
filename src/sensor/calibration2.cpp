#include "Morai_Woowa/calibration2.h"

calibration2::calibration2(ros::NodeHandle& nh)
{
    lidar_sub = nh.subscribe("lidar_pre", 1, &calibration2::lidar_callBack, this);
    object_sub = nh.subscribe("person", 1, &calibration2::object_callBack, this);
    points_array_pub = nh.advertise<morai_woowa::average_points_array>("average_points",10);
    min_distance = 987654321.0;    // 최소 거리 갱신

    ROS_INFO("Calibration 2!");

    this->do_cali();
}

void calibration2::lidar_callBack(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    last_lidar_time = msg->header.stamp; // 라이다 스탬프

    pcl::PCLPointCloud2 cloud_intermediate; // 포인트 클라우드 데이터 저장

    // 포인터가 가리키는 포인트 클라우드 데이터를 cloud_intermediate에 저장
    pcl_conversions::toPCL(*msg, cloud_intermediate);
    // cloud_intermediate에 저장된 PCL 포인트 클라우드 데이터를 cloud 객체로 변환
    pcl::fromPCLPointCloud2(cloud_intermediate, cloud);

    // std::vector<cv::Point3f>로 변환
    std::vector<cv::Point3f> objectPoints;
    std::map<int, std::vector<cv::Point3f>> classPoints;    // 클래스별 라이다 좌표 저장

    // 포인트 변환
    for (size_t i = 0; i < cloud.size(); ++i) {
        int classId = static_cast<int>(cloud.points[i].intensity); // intensity를 클래스 ID로 변환
        classPoints[classId].emplace_back(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
        // intensity.emplace_back(cloud.points[i].intensity);
    }
    
    lidar_points = classPoints;
}


void calibration2::object_callBack(const morai_woowa::obj_array::ConstPtr& msg)
{
    // 새로운 이미지 메시지를 큐에 추가
    img_msg_queue.push_back(msg);

    // 큐의 크기를 제한 (최대 10개의 메시지 유지)
    if (img_msg_queue.size() > 10) {
        img_msg_queue.pop_front();
    }

    // 큐에서 last_lidar_time과 가장 가까운 이미지 메시지 찾기
    auto closest_msg = std::min_element(img_msg_queue.begin(), img_msg_queue.end(),
        [this](const morai_woowa::obj_array::ConstPtr& a, const morai_woowa::obj_array::ConstPtr& b) {
            return std::abs((a->header.stamp - last_lidar_time).toSec()) <
                   std::abs((b->header.stamp - last_lidar_time).toSec());
        });

    if (closest_msg != img_msg_queue.end()) {
        // 가장 가까운 메시지를 사용하여 처리
        frame = cv::Mat((*closest_msg)->image.height, (*closest_msg)->image.width, CV_8UC3, const_cast<unsigned char*>((*closest_msg)->image.data.data()), (*closest_msg)->image.step);

        // BoundingBox를 저장할 벡터 선언
        std::vector<BoundingBox> bounding_boxes;

        for (size_t i = 0; i < (*closest_msg)->objects.size(); i++) {
            // 사람이면 BoundingBox 저장
            if ((*closest_msg)->objects[i].name == "person") {
                BoundingBox box;
                box.ymax = (*closest_msg)->objects[i].ymax;
                box.ymin = (*closest_msg)->objects[i].ymin;
                box.xmax = std::min((*closest_msg)->objects[i].xmax+20, 640);
                box.xmin = std::max((*closest_msg)->objects[i].xmin-20, 0);
                bounding_boxes.push_back(box);
            }
        }

        // 가장 가까운 이미지로 projection 함수 호출
        this->projection(frame, bounding_boxes);
    }
}

void calibration2::do_cali()
{
    // 카메라 내부 매트릭스, 회전 벡터, 변환 벡터 정의
    cameraMatrix = (cv::Mat_<double>(3, 3) << fx, 0, cx,
                                            0,fy, cy,
                                            0, 0, 1);
    // 라이다 -> 카메라 좌표계로 일치시키는 행렬
    rvec = (cv::Mat_<double>(3, 3) << 0, 1, 0,
                                     0, 0, 1,
                                     -1, 0, 0);

    tvec = (cv::Mat_<double>(3, 1) << lidar_x - camera_x, lidar_y - camera_y, lidar_z - camera_z); 
    
    distCoeffs = cv::Mat::zeros(4, 1, CV_64F); // 왜곡 없음

    std::cout << "cameraMatrix: \n" << cameraMatrix << "\n\n";
    std::cout << "rvec: \n" << rvec << "\n\n";
    std::cout << "tvec: \n" << tvec << "\n\n";
    std::cout << "distCoeffs: \n" << distCoeffs << "\n\n";
}


void calibration2::projection(cv::Mat frame, std::vector<BoundingBox> bounding_boxes)
{
   try {
        // lidar_points가 존재했을 때 실행
        if (!lidar_points.empty()) {

            morai_woowa::average_points_array msg;

            // 각 클래스에 대해 클러스터 크기 필터링 및 평균 계산
            for (const auto& pair : lidar_points) {
                int classId = pair.first;
                const std::vector<cv::Point3f>& points = pair.second;

                if (points.empty() || points.size() < 10) {
                    continue; // 포인트가 없는 클래스는 건너뜁니다
                }

                // 최소 및 최대 좌표값 계산
                float minX = std::numeric_limits<float>::max();
                float maxX = std::numeric_limits<float>::lowest();
                float minY = std::numeric_limits<float>::max();
                float maxY = std::numeric_limits<float>::lowest();
                float minZ = std::numeric_limits<float>::max();
                float maxZ = std::numeric_limits<float>::lowest();

                // 각 클러스터의 x, y, z 최소/최대값 계산
                for (const auto& point : points) {
                    minX = std::min(minX, point.x);
                    maxX = std::max(maxX, point.x);
                    minY = std::min(minY, point.y);
                    maxY = std::max(maxY, point.y);
                    minZ = std::min(minZ, point.z);
                    maxZ = std::max(maxZ, point.z);
                }

                // 각 축의 길이 계산
                float lengthX = maxX - minX;
                float lengthY = maxY - minY;
                float lengthZ = maxZ - minZ;

                // 클러스터 크기 필터링 
                if (lengthX > 1 || lengthY > 1 || 0.3 > lengthZ || lengthZ > 1.8) {
                    continue; // 클러스터가 크면 pass
                }

                // 클러스터 평균점 계산
                cv::Point3f average = {0.0, 0.0, 0.0};
                for (const auto& point : points) {
                    average.x += point.x;
                    average.y += point.y;
                    average.z += point.z;
                }

                average.x /= points.size();
                average.y /= points.size();
                average.z /= points.size();

                // 이미지 포인트를 저장할 벡터
                std::vector<cv::Point2f> imagePoints;
                std::vector<cv::Point3f> mean = { average };
                std::cout << average.x << "," << average.y <<"," << average.z <<std::endl;
                // 3D 포인트를 2D 이미지 평면으로 투영
                cv::projectPoints(mean, rvec, tvec, cameraMatrix, distCoeffs, imagePoints);

                int x = static_cast<int>(imagePoints[0].x);
                int y = static_cast<int>(imagePoints[0].y);

                // 클러스터 평균점이 바운딩 박스 안에 있는지 확인
                bool isInBoundingBox = false;
                for (const auto& box : bounding_boxes) {
                    if ((box.xmin <= x && x <= box.xmax) &&
                        (box.ymin <= y && y <= box.ymax)) {
                        isInBoundingBox = true;
                        break;
                    }
                }

                // 평균점이 바운딩 박스 안에 있으면 사람으로 판단
                if (isInBoundingBox) {
                    cv::circle(frame, cv::Point(x, y), 5, cv::Scalar(0, 255, 0), -1); // 평균점 표시

                    // 평균 거리 계산
                    double averageDistance = std::sqrt(average.x * average.x +
                                                       average.y * average.y +
                                                       average.z * average.z);

                    morai_woowa::average_points points_msg;
                    points_msg.average_x = average.x;
                    points_msg.average_y = average.y;
                    points_msg.average_z = average.z;
                    points_msg.distance = averageDistance;

                    msg.points_array.push_back(points_msg);
                }
            }

            // average point publish
            points_array_pub.publish(msg);
                        // 이미지를 출력
            cv::imshow("Projected Points", frame);
            cv::waitKey(1);  // 1ms 대기, OpenCV 윈도우를 유지
        }
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("Calibration ERROR! %s", e.what());
    }
}