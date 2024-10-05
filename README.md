# Morai_Woowa
R-BIZ Challenge:배달의 민족 로봇배달 챌린지

### /current_pose 생성
rosrun Morai_Woowa location.py 

### way_point 저장
rosrun Morai_Woowa waypoint_saver.py 

path폴더 안에 생성됨. 파일 이름 바꾸기!!

### state_node 실행
rosrun Morai_Woowa state_node 

로드할 경로 이름 바꾸기!!


# Topics
name : /current_pose

type : geomtry_msgs.Pose2D

설명 : utm_k좌표계 기준 robot의 x,y,yaw 


name : /lidar_pre

type : sensor_msgs/PointCloud2

설명 : roi, voxel, ransac 전터리된 라이다값


name : /lidar_utm

type : sensor_msgs/PointCloud2

설명 : 전처리된 pcl데이터를 utm_k 좌표계로 좌표계 변경 
