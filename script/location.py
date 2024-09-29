#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Pose2D
from morai_msgs.msg import GPSMessage
import tf
import pyproj

class GPSToUTM:
    def __init__(self):
        rospy.init_node('gps_to_utm', anonymous=True)

        # Publishers and Subscribers
        self.pose_pub = rospy.Publisher('current_pose', Pose2D, queue_size=10)
        self.gps_sub = rospy.Subscriber('/gps', GPSMessage, self.gps_callback)
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback)

        # Proj 초기화
        self.proj_WGS84 = pyproj.Proj(init='epsg:4326')  # WGS84
        self.proj_UTMK = pyproj.Proj(init='epsg:5179')   # UTM-K

        self.current_yaw = 0.0

    def gps_callback(self, msg):
        latitude = msg.latitude
        longitude = msg.longitude

        x, y = self.lat_lon_to_utmk(longitude, latitude)

        if x is not None and y is not None:
            pose = Pose2D()
            pose.x = x
            pose.y = y
            pose.theta = self.current_yaw

            self.pose_pub.publish(pose)
            rospy.loginfo("Published UTM Coordinates: (%.2f, %.2f) with Yaw: %.2f", pose.x, pose.y, pose.theta)
        else:
            rospy.logwarn("Failed to convert lat/lon to UTM.")

    def imu_callback(self, msg):
        # Quaternion에서 yaw 추출
        quaternion = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.current_yaw = euler[2]  # yaw 값

    def lat_lon_to_utmk(self, lon, lat):
        try:
            x,y = pyproj.transform(self.proj_WGS84,self.proj_UTMK,lon,lat)
            return x, y
        except Exception as e:
            rospy.logerr("Projection error: %s", str(e))
            return None, None

if __name__ == '__main__':
    try:
        gps_to_utm = GPSToUTM()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
