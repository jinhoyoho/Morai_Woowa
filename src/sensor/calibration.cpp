#include "Morai_Woowa/calibration.h"
#include "Morai_Woowa/LiDAR_pre.h"

calibration::calibration()
{
    ros::NodeHandle nh;
    image_sub = nh.subscribe("python_image", 1, &calibration::image_callBack, this);
    lidar_sub = nh.subscribe("lidar_pre", 1, &calibration::lidar_callBack, this);

    // camera 내부 파라미터 값 저장
    intrinsic << fx, skew_c, cx,
                 0, fy, cy,
                 0, 0, 1;

}

void calibration::image_callBack(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        // bgr data가 수신되어 그냥 이용해도 된다
        frame = cv::Mat(msg->height, msg->width, CV_8UC3, const_cast<unsigned char*>(msg->data.data()), msg->step);

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
    pcl::PCLPointCloud2* cloud_intermediate = new pcl::PCLPointCloud2;
    pcl::PointCloud<pcl::PointXYZ> cloud;   // pointcloud 저장

    // 포인터가 가리키는 포인트 클라우드 데이터를 cloud_intermediate에 저장
    pcl_conversions::toPCL(*msg, *cloud_intermediate);
    // cloud_intermediate에 저장된 PCL 포인트 클라우드 데이터를 cloud 객체로 변환
    pcl::fromPCLPointCloud2(*cloud_intermediate, cloud);

    Eigen::MatrixXd points(cloud.size(), 3); // x, y, z

    for (size_t i = 0; i < cloud.size(); ++i) {
        points(i, 0) = cloud.points[i].x;       // x
        points(i, 1) = cloud.points[i].y;       // y
        points(i, 2) = cloud.points[i].z;       // z

    }

    lidar_points = points;  // 변수에 저장
}

void calibration::do_cali()
{


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