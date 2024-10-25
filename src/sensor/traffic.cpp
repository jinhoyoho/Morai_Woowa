#include "Morai_Woowa/traffic.h"

// 빨간불일때 false, 나머진 true
// 395.27, -111.38

Traffic::Traffic(ros::NodeHandle& nh)
{
    flag = true; // flag false로 생성
    count = 0;
    image_sub = nh.subscribe("traffic_image", 1, &Traffic::image_callBack, this);
    pose_sub = nh.subscribe("current_pose", 10, &Traffic::pose_callback, this);
    flag_pub = nh.advertise<std_msgs::Bool>("traffic", 10);
    first_false_flag = true;
    first_false_time = ros::Time::now();
    std::cout << "Traffic on" << "\n";
}

void Traffic::pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    // Extract position
    current_x = msg->pose.position.x;
    current_y = msg->pose.position.y;

    if(ros::Time::now() - first_false_time > ros::Duration(25) )
        flag = true;
    
    std_msgs::Bool flag_msg;
    flag_msg.data = flag;
    flag_pub.publish(flag_msg);
}

void Traffic::image_callBack(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        float distance = std::sqrt((current_x - 395.27)*(current_x - 395.27) 
                                  + (current_y + 111.38)*(current_y + 111.38));
        ROS_INFO("Distance: %f", distance);

        if(distance < 2)    // 1 meter일때
        {
            ROS_INFO("Traffic ON!");
            if(msg)
            {
                frame = cv::Mat(msg->height, msg->width, CV_8UC3, const_cast<unsigned char*>(msg->data.data()), msg->step);
                cv::imshow("traffic image", frame);
                if (cv::waitKey(10) == 27) exit(1);  // esc키로 종료
                this->process_image();
            }
        }
        else
        {
            flag = true;    // 거리가 멀면 무조~~~건 true
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
        cv::Mat hsvImage_red;
        cv::Mat hsvImage_green;
        cv::cvtColor(frame, hsvImage_red, cv::COLOR_BGR2HSV);
        cv::cvtColor(frame, hsvImage_green, cv::COLOR_BGR2HSV);

        // 초록색 범위 정의 (HSV)
        cv::Scalar lowerGreen(40, 15, 170); // 낮은 초록색 범위
        cv::Scalar upperGreen(70, 70, 255); // 높은 초록색 범위

        cv::Scalar lowerRed(0, 0, 70); // 낮은 빨간색 범위
        cv::Scalar upperRed(25, 160, 255); // 높은 빨간색 범위

        // 초록색 픽셀 마스크 생성
        cv::Mat greenMask;
        cv::inRange(hsvImage_green, lowerGreen, upperGreen, greenMask);

        cv::Mat redMask;
        cv::inRange(hsvImage_red, lowerRed, upperRed, redMask);


        // 전체 픽셀 개수
        int totalPixelCount = hsvImage_green.rows * hsvImage_green.cols;


        // 초록색 픽셀 수 카운트
        int greenPixelCount = cv::countNonZero(greenMask);
        // 초록색 비율 계산
        double greenRatio = static_cast<double>(greenPixelCount) / totalPixelCount;

        // 빨간색 픽셀 수 카운트
        int redPixelCount = cv::countNonZero(redMask);
        // 빨간색 비율 계산
        double redRatio = static_cast<double>(redPixelCount) / totalPixelCount;

        
        // 초록색이 더 크다면
        if(greenRatio > redRatio)   
        {
            mvf.push_back(GO);  // 출발 
            count++;    // 개수 증가
        }
        else if (redRatio > greenRatio)
        {
            mvf.push_back(STOP);    //  멈춰!
            count++;
        }


        if (count >= 10)    // 10개가 쌓였다면
        {
            double sum = std::accumulate(mvf.begin(), mvf.end(), 0); // 초기값은 0
            sum = sum / count; // 개수만큼 나누기

            mvf.erase(mvf.begin());  // 맨 앞 요소 삭제
            count--;

            // 빨간불이면 false
            if(sum <= 0.5) // 5개미만
            {
                ROS_INFO("STOP!");
                if(flag == true)
                    first_false_time = ros::Time::now();

                flag = false;
                if(first_false_flag){
                    first_false_flag = false;
                }
            }
            else
            {
                // if(flag == true)
                //     first_false_time = ros::Duration(10000000000);

                ROS_INFO("GO!");
                flag = true;
            }
        }
    }

    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not crop image! %s", e.what());
    }
}

