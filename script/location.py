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

        # br = tf.TransformBroadcaster()

        # # GPS 좌표를 기반으로 gps_frame 발행
        # br.sendTransform((0, 0, 0), 
        #                     tf.transformations.quaternion_from_euler(0, 0, 0),  # 회전 (yaw, pitch, roll)
        #                     rospy.Time.now(),
        #                     "gps_frame",  # TF 프레임 이름
        #                     "map")        # 부모 프레임 이름

        # # LiDAR 프레임 변환 (예: LiDAR가 gps_frame을 기준으로 offset이 있을 경우)
        # lidar_translation = (5, 0.0, 0.0)  # LiDAR의 위치 오프셋 (예시)
        # lidar_rotation = tf.transformations.quaternion_from_euler(0, 0, 0)  # 회전 (예시)

        # # LiDAR 프레임 발행
        # br.sendTransform(lidar_translation,
        #                     lidar_rotation,
        #                     rospy.Time.now(),
        #                     "velodyn_points",  # TF 프레임 이름
        #                     "gps_frame")    # 부모 프레임 이름


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
