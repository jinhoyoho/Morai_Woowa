#include "Morai_Woowa/traffic.h"

Traffic::Traffic()
{
    flag = false; // flag false로 생성
    ros::NodeHandle nh;
    image_sub = nh.subscribe("python_image", 1, &Traffic::image_callBack, this);
    object_sub = nh.subscribe("detected_object", 1, &Traffic::object_callBack, this);
    std::cout << "Traffic on" << "\n";
}

void Traffic::object_callBack(const Morai_Woowa::obj_info::ConstPtr& msg)
{
    std::string class_name = msg->name;

    if (class_name == "traffic light")  // 신호등 일때만 처리
    {
        int vert = std::abs(msg->ymin - msg->ymax); // 세로
        int hori = std::abs(msg->xmax - msg->xmin); // 가로
        
        if(vert > hori) // 가로보다 세로가 더 길어야 함
        {
            this->process_image(msg->xmin, msg->xmax, msg->ymin, msg->ymax);
        }
    }
}


void Traffic::image_callBack(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        // bgr data가 수신되어 그냥 이용해도 된다
        frame = cv::Mat(msg->height, msg->width, CV_8UC3, const_cast<unsigned char*>(msg->data.data()), msg->step);

        // 변환된 이미지를 보여줍니다.
        // cv::imshow("Received Image", frame);
        // if (cv::waitKey(10) == 27) exit(1);  // esc키로 종료  
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert image! %s", e.what());
        ROS_ERROR("Received image encoding: %s", msg->encoding.c_str());
    }
}

void Traffic::process_image(int xmin, int xmax, int ymin, int ymax)
{
    try{
        cv::Rect rect(xmin, ymin,  std::abs(xmax - xmin), std::abs(ymin - ymax)); // X, Y, W, H
        cv::Mat subImage = frame(rect); // 자르기

        // RGB로 색상 구분하기

        // // BGR 이미지를 HSV로 변환
        // cv::Mat hsvImage;
        // cv::cvtColor(subImage, hsvImage, cv::COLOR_BGR2HSV);

        // // 초록색 범위 정의 (HSV)
        // cv::Scalar lowerGreen(35, 100, 100); // 낮은 초록색 범위
        // cv::Scalar upperGreen(85, 255, 255); // 높은 초록색 범위

        // // 초록색 픽셀 마스크 생성
        // cv::Mat greenMask;
        // cv::inRange(hsvImage, lowerGreen, upperGreen, greenMask);

        // // 초록색 픽셀 수 카운트
        // int greenPixelCount = cv::countNonZero(greenMask);
        // int totalPixelCount = subImage.rows * subImage.cols;

        // // 초록색 비율 계산
        // double greenRatio = static_cast<double>(greenPixelCount) / totalPixelCount;


         // 초록색 범위 정의 (RGB)
        int lowerGreenR = 0;   // Red 최소값
        int lowerGreenG = 100; // Green 최소값
        int lowerGreenB = 0;   // Blue 최소값

        int upperGreenR = 100; // Red 최대값
        int upperGreenG = 255; // Green 최대값
        int upperGreenB = 100; // Blue 최대값

        // 초록색 픽셀 수 카운트
        int greenPixelCount = 0;
        int totalPixelCount = subImage.rows * subImage.cols;

        // RGB 이미지에서 초록색 픽셀 수 계산
        for (int y = 0; y < subImage.rows; ++y) {
            for (int x = 0; x < subImage.cols; ++x) {
                cv::Vec3b color = subImage.at<cv::Vec3b>(y, x); // 픽셀 색상 가져오기
                // 초록색 범위 확인
                if (color[2] >= lowerGreenR && color[1] >= lowerGreenG && color[0] <= upperGreenB &&
                    color[2] <= upperGreenR && color[1] <= upperGreenG && color[0] >= lowerGreenB) {
                    greenPixelCount++;
                }
            }
        }

        // 초록색 비율 계산
        double greenRatio = static_cast<double>(greenPixelCount) / totalPixelCount;

        cv::imshow("Traffic Image", subImage);
        if (cv::waitKey(10) == 27) exit(1);  // esc키로 종료
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not crop image! %s", e.what());
    }
}

