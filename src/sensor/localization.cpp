#include <ros/ros.h>
#include <tf/tf.h>  
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose2D.h>
#include <proj.h>  // proj 라이브러리 포함

class GPSToUTM {
public:
    GPSToUTM() {
        pose_pub_ = nh_.advertise<geometry_msgs::Pose2D>("current_pose", 10);
        gps_sub_ = nh_.subscribe("/gps", 10, &GPSToUTM::gpsCallback, this);
        imu_sub_ = nh_.subscribe("/imu", 10, &GPSToUTM::imuCallback, this);

        // Proj 초기화
        proj_WGS84 = proj_create_crs_to_crs(PJ_DEFAULT_CTX, "EPSG:4326", "EPSG:5179", nullptr);
        proj_UTMK = proj_create_crs_to_crs(PJ_DEFAULT_CTX, "EPSG:5179", "EPSG:5179", nullptr);
    }

    ~GPSToUTM() {
        proj_destroy(proj_WGS84);
        proj_destroy(proj_UTMK);
    }

    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        double latitude = msg->latitude;
        double longitude = msg->longitude;

        double easting, northing;
        if (latLonToUTMK(longitude, latitude, easting, northing)) {
            geometry_msgs::Pose2D pose;
            pose.x = easting;
            pose.y = northing;
            pose.theta = current_yaw_;

            pose_pub_.publish(pose);
            ROS_INFO("Published UTM Coordinates: (%.2f, %.2f) with Yaw: %.2f", pose.x, pose.y, pose.theta);
        } else {
            ROS_WARN("Failed to convert lat/lon to UTM.");
        }
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        double roll, pitch, yaw;
        tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
        
        current_yaw_ = yaw;
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher pose_pub_;
    ros::Subscriber gps_sub_;
    ros::Subscriber imu_sub_;
    double current_yaw_ = 0.0;

    PJ* proj_WGS84;
    PJ* proj_UTMK;

    bool latLonToUTMK(double lon, double lat, double& easting, double& northing) {
        PJ_COORD coord = proj_coord(lon, lat, 0, 0);
        PJ_COORD utm_coord = proj_trans(proj_WGS84, PJ_FWD, coord);
        
        easting = utm_coord.xy.x;
        northing = utm_coord.xy.y;

        return true;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gps_to_utm");

    GPSToUTM gps_to_utm;

    ros::spin();
    return 0;
}
