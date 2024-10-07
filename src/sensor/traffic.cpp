#include "Morai_Woowa/traffic.h"

Traffic::Traffic()
{
    flag = false; // flag false로 생성
    img_flag = false; // image flag
    ros::NodeHandle nh;
    image_sub = nh.subscribe("traffic_image", 1, &Traffic::image_callBack, this);
    std::cout << "Traffic on" << "\n";
}


void Traffic::image_callBack(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        if(msg)
        {
            frame = cv::Mat(msg->height, msg->width, CV_8UC3, const_cast<unsigned char*>(msg->data.data()), msg->step);
            cv::imshow("traffic image", frame);
            if (cv::waitKey(10) == 27) exit(1);  // esc키로 종료
            this->process_image();
        }
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert image! %s", e.what());
        ROS_ERROR("Received image encoding: %s", msg->encoding.c_str());
    }
}

void Traffic::process_image()
{
    try{
        // BGR 이미지를 HSV로 변환
        cv::Mat hsvImage;
        cv::cvtColor(frame, hsvImage, cv::COLOR_BGR2HSV);

        // 초록색 범위 정의 (HSV)
        cv::Scalar lowerGreen(35, 100, 100); // 낮은 초록색 범위
        cv::Scalar upperGreen(85, 255, 255); // 높은 초록색 범위

        // 초록색 픽셀 마스크 생성
        cv::Mat greenMask;
        cv::inRange(hsvImage, lowerGreen, upperGreen, greenMask);

        // 초록색 픽셀 수 카운트
        int greenPixelCount = cv::countNonZero(greenMask);
        int totalPixelCount = frame.rows * frame.cols;

        // 초록색 비율 계산
        double greenRatio = static_cast<double>(greenPixelCount) / totalPixelCount;

        // // RGB로 색상 구분하기

        // // 초록색 범위 정의 (RGB)
        // int lowerGreenR = 0;   // Red 최소값
        // int lowerGreenG = 100; // Green 최소값
        // int lowerGreenB = 0;   // Blue 최소값

        // int upperGreenR = 100; // Red 최대값
        // int upperGreenG = 255; // Green 최대값
        // int upperGreenB = 200; // Blue 최대값

        // // 초록색 픽셀 수 카운트
        // int greenPixelCount = 0;
        // int totalPixelCount = frame.rows * frame.cols;

        // // RGB 이미지에서 초록색 픽셀 수 계산
        // for (int y = 0; y < frame.rows; ++y) {
        //     for (int x = 0; x < frame.cols; ++x) {
        //         cv::Vec3b color = frame.at<cv::Vec3b>(y, x); // 픽셀 색상 가져오기
        //         // 초록색 범위 확인
        //         if (color[2] >= lowerGreenR && color[1] >= lowerGreenG && color[0] <= upperGreenB &&
        //             color[2] <= upperGreenR && color[1] <= upperGreenG && color[0] >= lowerGreenB) {
        //             greenPixelCount++;
        //         }
        //     }
        // }

        // // 초록색 비율 계산
        // double greenRatio = static_cast<double>(greenPixelCount) / totalPixelCount;

        // std::cout << greenRatio << "\n";

        if (greenRatio > 0) // 초록불이면 가라
        {
            flag = true;    // 고고고 -> topic으로 주고받는게 나을 것 같음
            ROS_INFO("GO!");
        }
        else
        {
            flag = false;
            ROS_INFO("STOP!");
        }
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not crop image! %s", e.what());
    }
}

