# Morai_Woowa
R-BIZ Challenge:배달의 민족 로봇배달 챌린지

### /current_pose 생성
rosrun morai_woowa location.py 

### way_point 저장
rosrun morai_woowa waypoint_saver.py 
<br/><br/>
path폴더 안에 생성됨. 파일 이름 바꾸기!!

### state_node 실행
roslaunch morai_woowa state.launch
<br/>
or
<br/>
rosrun morai_woowa state_node 
<br/><br/>
코드 혹은 런치파일에 로드할 경로 이름 바꾸기!!

### visual 실행
roslaunch morai_woowa visual.launch
<br/><br/>
런치파일에 로드할 경로 이름 바꾸기!!

### rosbridge 실행
sudo apt-get install ros-noetic-rosbridge-server
<br/><br/>
roscd rosbridge_server/launch (파일찾기)
<br/><br/>
gedit rosbridge_websocket.launch
<br/><br/>
192.168.0.2 으로 설정(예시)
<arg name="address" default="192.168.0.2" /> 이 부분 바꾸기
<br/><br/>
roslaunch rosbridge_server rosbridge_websocket.launch - rosbridge 실행

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
<br/>
name : /obstacle<br/>
type : sensor_msgs/PointCloud2<br/>
설명 : 장해물의 위치와 속도를 나타내는 토픽 x,y는 장애물의 중심 좌표이고 z, intensity는 각각 장애물의 x,y축 속도이다.<br/>
<br/>
name : /visualization_marker_array<br/>
type : visualization_msgs/MarkerArray<br/>
설명 : obstacle 토픽의 장애물 벡터를 화살표로 나타낸 시각화용 토픽<br/>
<br/>
name : /candidate_path<br/>
type : sensor_msgs/PointCloud<br/>
설명 : dwa가 생성한 후보경로, 시각화용<br/>
<br/>
name : /president_path<br/>
type : sensor_msgs/PointCloud<br/>
설명 : dwa로 결정한 최적 경로. local path가 된다.<br/>
