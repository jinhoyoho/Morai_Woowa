#!/usr/bin/env python3
 
import rospy
import cv2
import numpy as np
import os, rospkg

from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from cv_bridge import CvBridgeError, CvBridge
from ultralytics import YOLO

from Morai_Woowa.msg import obj_info

class IMGParser:
    def __init__(self):
        rospy.init_node('image_parser', anonymous=True)
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        self.image_pub = rospy.Publisher('python_image', Image, queue_size=10)
        self.obj_pub = rospy.Publisher('detected_object', obj_info, queue_size=10)
        self.is_image = False
        self.br = CvBridge()

        # file의 directory 경로
        current_dir = os.path.dirname(os.path.abspath(__file__))
        model_path = os.path.join(current_dir, "yolov10n.pt")

        # YOLO 모델을 현재 경로에서 불러옵니다.
        model = YOLO(model_path)

        rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            os.system('clear')
            if not self.is_image:
                print("[1] can't subscribe '/image_jpeg/compressed' topic... \n    please check your Camera sensor connection")
                
            else: # 센서 연결시
                results = model(self.img_bgr)
                
                #object_detection결과가 bbox에 담긴다
                boxes = results[0].boxes.cpu().numpy().data
            
                for box in boxes:
                    label = results[0].names[int(box[5])]
                    if box[4] > 0.3 and (label == 'person' or label == 'traffic light'): # 사람 or 신호등 경우에만 그리기
                        detected_obj = obj_info()
                        left = int(box[0])
                        bottom = int(box[1])
                        right = int(box[2])
                        top = int(box[3])
                        confidence = int(box[4])
                      
                        #메시지에 xmin, xmax, ymin, ymax 입력해준다.
                        detected_obj.xmin = left
                        detected_obj.ymin = bottom
                        detected_obj.xmax = right
                        detected_obj.ymax = top
                        detected_obj.name = label

                        # 인지된 객체 바운딩박스 그려준다.
                        self.img_bgr = cv2.rectangle(self.img_bgr, (left, bottom), (right, top), (0,0,255),2)
                        self.obj_pub.publish(detected_obj)

                # cv2.imshow("Image window", self.img_bgr)
                # if cv2.waitKey(1) == ord('q'):
                #     break
                # success = cv2.imwrite('./test.jpg', self.img_bgr)
                # if success:
                #     print("Image saved successfully!")
                # else:
                #     print("Failed to save the image!")

                # publish
                self.image_pub.publish(self.br.cv2_to_imgmsg(self.img_bgr))


            rate.sleep()


    def callback(self, msg):
        self.is_image = True
        np_arr = np.frombuffer(msg.data, np.uint8) # msg를 uint8 형태로 변환
        self.img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # 3차원 (rgb)형태로 전환
        

if __name__ == '__main__':
    try:
        IMGParser = IMGParser()
    except rospy.ROSInterruptException:
        pass