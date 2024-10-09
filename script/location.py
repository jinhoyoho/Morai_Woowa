#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from morai_msgs.msg import GPSMessage
from sensor_msgs.msg import Imu
from pyproj import Proj
import tf
import math

class GPSToUTM:
    def __init__(self):
        rospy.init_node('gps_to_utm', anonymous=True)

        # Publishers and Subscribers
        self.pose_pub = rospy.Publisher('/current_pose', PoseStamped, queue_size=10)
        self.gps_sub = rospy.Subscriber('/gps', GPSMessage, self.gps_callback)
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback)

        # Proj 초기화
        self.proj_UTM = Proj(proj='utm', zone=52, ellps = 'WGS84', preserve_units=False)
        
        # TF 브로드캐스터 초기화
        self.br = tf.TransformBroadcaster()
        
        # 현재 쿼터니언 초기화
        self.current_orientation = [0.0, 0.0, 0.0, 1.0]  # 초기 쿼터니언 (0, 0, 0, 1)

    def lat_lon_to_utmk(self, lon, lat, x_off, y_off):
        # 위도, 경도를 UTM-K 좌표로 변환
        utm_xy = self.proj_UTM(lon, lat)        
        return utm_xy[0]-x_off, utm_xy[1]-y_off

    def gps_callback(self, msg):
        latitude = msg.latitude
        longitude = msg.longitude
        x_offset = msg.eastOffset
        y_offset = msg.northOffset

        x, y = self.lat_lon_to_utmk(longitude, latitude, x_offset, y_offset)

        if x is not None and y is not None:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "robot"  # 기준 좌표계 설정
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            pose_stamped.pose.position.z = 0  # Z축은 0으로 설정

            # IMU에서 얻은 쿼터니언 사용
            pose_stamped.pose.orientation.x = self.current_orientation[0]
            pose_stamped.pose.orientation.y = self.current_orientation[1]
            pose_stamped.pose.orientation.z = self.current_orientation[2]
            pose_stamped.pose.orientation.w = self.current_orientation[3]

            self.pose_pub.publish(pose_stamped)
            rospy.loginfo("Published UTM Coordinates: (%.2f, %.2f) with Orientation: (%.2f, %.2f, %.2f, %.2f)", 
                          pose_stamped.pose.position.x, pose_stamped.pose.position.y,
                          self.current_orientation[0], self.current_orientation[1],
                          self.current_orientation[2], self.current_orientation[3])

            # TF 발행
            self.publish_tf(x, y, self.current_orientation)

        else:
            rospy.logwarn("Failed to convert lat/lon to UTM.")

    def publish_tf(self, x, y, orientation):
        # 현재 시간
        current_time = rospy.Time.now()

        # TF 발행
        self.br.sendTransform((x, y, 0), orientation, current_time, "robot", "map")

    def imu_callback(self, msg):
        # IMU 데이터에서 쿼터니언 업데이트
        self.current_orientation = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ]

if __name__ == '__main__':
    gps_to_utm = GPSToUTM()
    rospy.spin()
