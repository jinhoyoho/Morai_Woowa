#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import os, rospkg

from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from cv_bridge import CvBridgeError, CvBridge
from ultralytics import YOLO
from morai_woowa.msg import obj_info2, obj_array3

class IMGParser:
    def __init__(self):
        self.is_image1 = False
        self.is_image2 = False
        self.img_stamp1 = 0
        self.img_stamp2 = 0
        self.img_bgr1 = None
        self.img_bgr2 = None

        rospy.init_node('image_parser', anonymous=True)
        self.image_sub1 = rospy.Subscriber("/image_jpeg/compressed1", CompressedImage, self.callback1)
        self.image_sub2 = rospy.Subscriber("/image_jpeg/compressed2", CompressedImage, self.callback2)
        self.traffic_image_pub = rospy.Publisher('traffic_image', Image, queue_size=10)
        self.obj_pub1 = rospy.Publisher('person1', obj_array3, queue_size=10)
        self.obj_pub2 = rospy.Publisher('person2', obj_array3, queue_size=10)
  
        self.br = CvBridge()

        # file의 directory 경로
        current_dir = os.path.dirname(os.path.abspath(__file__))
        model_path = os.path.join(current_dir, "yolov10m.pt")

        # YOLO 모델을 현재 경로에서 불러옵니다.
        self.model = YOLO(model_path)

        rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            os.system('clear')

            if not self.is_image1 or not self.is_image2:
                print("[1] can't subscribe '/image_jpeg/compressed' topic... \n    please check your Camera sensor connection")
            else:  # 센서 연결 시
                imgs = [(self.img_bgr1, self.img_stamp1, 1), (self.img_bgr2, self.img_stamp2, 2)]

                for image, time_stamp, flag in imgs:
                    array_msg = obj_array3()  # 배열 선언
                    array_msg.header.stamp = time_stamp # time stamp 저장
                    array_msg.type = flag # 몇 번 째 이미지인지 확인

                    results = self.model(image)

                    image_copy = image

                    # object_detection 결과가 boxes에 담긴다
                    boxes = results[0].boxes.cpu().numpy().data

                    for box in boxes:
                        label = results[0].names[int(box[5])]

                        if box[4] > 0.3 and label == 'person':  # 사람인 경우
                            detected_obj = obj_info2()  # publish할 obj_info
                            left = int(box[0])
                            bottom = int(box[1])
                            right = int(box[2])
                            top = int(box[3])

                            # 메시지에 xmin, xmax, ymin, ymax 입력해준다.
                            detected_obj.xmin = left
                            detected_obj.ymin = bottom
                            detected_obj.xmax = right
                            detected_obj.ymax = top
                            detected_obj.name = label

                            array_msg.objects.append(detected_obj)  # 배열에 저장
                            image_copy = cv2.rectangle(image_copy, (detected_obj.xmin, detected_obj.ymin), (detected_obj.xmax, detected_obj.ymax), (0, 0, 255), 2)

                        elif box[4] > 0.3 and label == 'traffic light':
                            left = int(box[0])
                            bottom = int(box[1])  # top보다 더 작음
                            right = int(box[2])
                            top = int(box[3])

                            # 가로가 더 긴 신호등은 건너뛰기
                            if abs(left - right) > abs(bottom - top):
                                continue

                            # image를 crop해서 전달
                            if bottom < top and left < right:  # 유효한 크롭 범위 확인
                                traffic_image_copy = image_copy[bottom:top, left:right]
                            else:
                                rospy.logwarn("Invalid crop area for traffic light.")

                            # 이미지 크기 확인
                            if traffic_image_copy.size == 0 or traffic_image_copy.shape[0] == 0 or traffic_image_copy.shape[1] == 0:
                                rospy.logwarn("Empty image copy. Skipping publishing.")
                                continue  # 이미지가 유효하지 않으면 건너뜀

                            self.traffic_image_pub.publish(self.br.cv2_to_imgmsg(traffic_image_copy))

                    if image_copy.size == 0 or image_copy.shape[0] == 0 or image_copy.shape[1] == 0:
                        rospy.logwarn("Empty image copy. Skipping publishing.")
                        continue  # 이미지가 유효하지 않으면 건너뜀

                    # 이미지는 한 번만 보내기
                    array_msg.image = self.br.cv2_to_imgmsg(image_copy)

                    if flag == 1:
                        # # 배열과 이미지 보내기
                        # image = results[0].plot()
                        # cv2.imshow("Image window1", image)
                        # if cv2.waitKey(1) == ord('q'):
                        #     break

                        self.obj_pub1.publish(array_msg)
                        
                    elif flag == 2:
                        # # 배열과 이미지 보내기
                        # image = results[0].plot()
                        # cv2.imshow("Image window2", image)
                        # if cv2.waitKey(1) == ord('q'):
                        #     break

                        self.obj_pub2.publish(array_msg)

                        
            rate.sleep()

    def callback1(self, msg):
        # yaw -
        self.img_stamp1 = msg.header.stamp
        self.is_image1 = True
        np_arr1 = np.frombuffer(msg.data, np.uint8)  # msg를 uint8 형태로 변환
        self.img_bgr1 = cv2.imdecode(np_arr1, cv2.IMREAD_COLOR)  # 3차원 (RGB) 형태로 전환

    def callback2(self, msg):
        # yaw +
        self.img_stamp2 = msg.header.stamp
        self.is_image2 = True
        np_arr2 = np.frombuffer(msg.data, np.uint8)  # msg를 uint8 형태로 변환
        self.img_bgr2 = cv2.imdecode(np_arr2, cv2.IMREAD_COLOR)  # 3차원 (RGB) 형태로 전환

if __name__ == '__main__':
    try:
        IMGParser = IMGParser()
    except rospy.ROSInterruptException:
        pass
