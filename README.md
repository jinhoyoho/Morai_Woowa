# Morai_Woowa
R-BIZ Challenge:배달의 민족 로봇배달 챌린지

### /current_pose 생성
rosrun Morai_Woowa location.py 

### way_point 저장
rosrun Morai_Woowa waypoint_saver.py 
<br/><br/>
path폴더 안에 생성됨. 파일 이름 바꾸기!!

### state_node 실행
roslaunch Morai_Woowa state.launch
<br/>
or
<br/>
rosrun Morai_Woowa state_node 
<br/><br/>
코드 혹은 런치파일에 로드할 경로 이름 바꾸기!!

### visual 실행
roslaunch Morai_Woowa visual.launch
<br/><br/>
런치파일에 로드할 경로 이름 바꾸기!!


# Topics
name : /current_pose<br/>
type : geomtry_msgs.Pose2D<br/>
설명 : utm_k좌표계 기준 robot의 x,y,yaw<br/> 
<br/>
name : /lidar_pre<br/>
type : sensor_msgs/PointCloud2<br/>
설명 : roi, voxel, ransac 전처리된 라이다값<br/>
<br/>
name : /lidar_utm<br/>
type : sensor_msgs/PointCloud2<br/>
설명 : 전처리된 pcl데이터를 utm_k 좌표계로 좌표계 변경<br/>
