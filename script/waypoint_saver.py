#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose2D
import pandas as pd
import math
import matplotlib.pyplot as plt
import os

class PoseSaver:
    def __init__(self):
        rospy.init_node('pose_saver', anonymous=True)

        self.path_name = "test_path.csv"
        self.save_directory = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'path')

        self.pose_sub = rospy.Subscriber('current_pose', Pose2D, self.pose_callback)

        self.pose_data = []
        self.last_saved_pose = None
        self.index = 0

    def pose_callback(self, msg):
        current_pose = (msg.x, msg.y, msg.theta)

        if self.last_saved_pose is None:
            self.last_saved_pose = current_pose
            self.save_pose(current_pose)
            return

        distance = self.calculate_distance(self.last_saved_pose, current_pose)

        if distance >= 0.3:  # 0.3m 간격 
            self.last_saved_pose = current_pose
            self.save_pose(current_pose)

    def calculate_distance(self, pose1, pose2):
        return math.sqrt((pose2[0] - pose1[0]) ** 2 + (pose2[1] - pose1[1]) ** 2)

    def save_pose(self, pose):
        # pose는 (x, y, heading) 튜플
        self.pose_data.append((pose[0], pose[1], pose[2], self.index))
        self.index += 1
        rospy.loginfo("Saved Pose: x=%.2f, y=%.2f, heading=%.2f, index=%d", pose[0], pose[1], pose[2], self.index)

    def write_to_csv(self):
        path_name = os.path.join(self.save_directory, self.path_name)

        df = pd.DataFrame(self.pose_data, columns=['x', 'y', 'heading', 'index'])
        df.to_csv(path_name, index=False)
        rospy.loginfo("Data saved to pose_data.csv")
        self.plot_waypoints()  

    def plot_waypoints(self):
        if not self.pose_data:
            rospy.logwarn("No data to plot.")
            return
        
        x = [pose[0] for pose in self.pose_data]
        y = [pose[1] for pose in self.pose_data]

        plt.figure()
        plt.plot(x, y, marker='o', linestyle='-', color='b')
        plt.title("Waypoints")
        plt.xlabel("Easting (m)")
        plt.ylabel("Northing (m)")
        plt.grid()
        plt.axis('equal')
        # plt.savefig('waypoints_plot.png')  
        plt.show()  # 플롯 표시

if __name__ == '__main__':
    try:
        pose_saver = PoseSaver()
        rospy.spin()
        pose_saver.write_to_csv()  # 노드 종료 시 CSV 파일로 저장
    except rospy.ROSInterruptException:
        pass
