#!/usr/bin/env python3

import sys
import rospy
from sensor_msgs.msg import Image
import cv2 as cv 
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import time 
from geometry_msgs.msg import PoseStamped,Point
import math 
''' HSV Objekt detection green and blue 
    ## Green (hMin = 40 , sMin = 145, vMin = 146), (hMax = 90 , sMax = 255, vMax = 255)
    ## Blue ((hMin = 90 , sMin = 111, vMin = 150), (hMax = 130 , sMax = 234, vMax = 255)
----
Params:
    - lower_green = np.array([40, 80, 80])
    - upper_green = np.array([90, 255, 255])

    - lower_blue = np.array([90, 110, 150])
    - upper_blue = np.array([130, 230, 255])

----
Publisher:
    - /sphero_position_green    Pose
    - /sphero_position_blue     Pose 
Subcriber:
    - /pylon_camera_node/image_raw  Image 
    '''


class sphero_detection_hsv:

    def __init__(self):
        self.bridge = CvBridge()

        """
        self.video_cod = cv.VideoWriter_fourcc(*'XVID')

        frame_width =1278
        frame_height= 1278
        self.video_output= cv.VideoWriter('captured_video_3.avi',
                        self.video_cod,
                        10,
                        (frame_width,frame_height))"""

        self.desired_position_ = Point()
        self.desired_position_.x = 0
        self.desired_position_.y = 0
        self.desired_position_.z = 0

        self.rate = rospy.Rate(30)
        self.t0 = time.time()
        self.t1 = time.time()

        self.lower_green = np.array([0, 251, 169])
        self.upper_green = np.array([179, 255, 255])

        self.lower_red1 = np.array([140, 14, 86])
        self.upper_red1 = np.array([179, 255, 255])

        self.lower_red2 = np.array([160,0,0])
        self.upper_red2 = np.array([180,255,255])


        self.kernel= cv.getStructuringElement(cv.MORPH_CROSS, (3,3))

        self.image_sub = rospy.Subscriber("/pylon_camera_node/image_raw",Image,self.callback,queue_size=1)
        #self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.callback,queue_size=1)

        self.position_green_pub= rospy.Publisher("/turtlebot_position", PoseStamped, queue_size=1)
       # self.rate = rospy.Rate(10)

        self.green_alt  = PoseStamped() 
        self.green_alt.header.stamp = rospy.Time.from_sec(time.time())
        self.time  = PoseStamped() 
        self.time.header.stamp = rospy.Time.from_sec(time.time())
        self.green  = PoseStamped() 
        self.rate = rospy.Rate(35)

    def callback_point(self,data):
        self.desired_position_.x= data.x
        self.desired_position_.y = data.y

    def callback(self,data):

        self.green.header.stamp = rospy.Time.from_sec(time.time())
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        img = self.rescale_frame(cv_image,45)

        cv.imshow('img', img)
        cv.waitKey(1)
        box_points_green , mask_green , centroid_green= self.find_sphero(img,self.lower_green,self.upper_green,self.kernel)

        
        if box_points_green is not None:
            centroid_red= self.find_point2(img[int(centroid_green[1]-30):int(centroid_green[1]+30),int(centroid_green[0]-30):int(centroid_green[0]+30)],self.lower_red1,self.upper_red1,self.lower_red2,self.upper_red2,self.kernel)
            if centroid_red is not None:
                centroid_red[0] = centroid_red[0] + int(centroid_green[0]-30)
                centroid_red[1] = centroid_red[1] + int(centroid_green[1]-30)
                cv.circle(img,(int(round(centroid_red[0])),int(round(centroid_red[1]))),1,(0,0,255),-1)
                cv.imshow('img', img)

                #phi = math.fmod(math.fmod(math.atan2( centroid_green[1]-centroid_red[1],centroid_green[0]-centroid_red[0]),2.0 * math.pi) + 2.0 * math.pi, 2.0 * math.pi) * 180 / math.pi
                phi = math.atan2( centroid_green[1]-centroid_red[1],centroid_green[0]-centroid_red[0]) 
                #phi = math.atan2( centroid_red[1]-centroid_green[1],centroid_red[0]-centroid_green[0]) * 180 / math.pi
                print("phi ",math.degrees(phi))
                self.green.pose.position.x = round(centroid_green[0])
                self.green.pose.position.y = round(centroid_green[1])
                self.green.pose.orientation.z = round(phi,4)
                cv.circle(img,(int(round(centroid_green[0])),int(round(centroid_green[1]))),1,(0,0,255),-1)
                # self.video_output.write(img)
                self.position_green_pub.publish(self.green)
            else:
                self.green.pose.position.x = -1
                self.green.pose.position.y = -1
                self.green.pose.orientation.z = -1
                print("no red color found ",-1)
                self.position_green_pub.publish(self.green)

        else:
            #print("nothing found")
            self.green.pose.position.x = -1
            self.green.pose.position.y = -1
            self.green.pose.orientation.z = -1
            print("no green color found ",-1)
            self.position_green_pub.publish(self.green)

        self.rate.sleep()
        cv.circle(img,(int(round(self.desired_position_.x)),int(round(self.desired_position_.y))),2,(0,255,255),-1)
        cv.imshow('mask_green', mask_green)
        cv.imshow('img', img)
        cv.waitKey(1)


    def find_sphero(self,img_rgb,lower,upper,kernel):
        #img_rgb = cv.GaussianBlur(img_rgb, (3,3),0)    
        img_hsv = cv.cvtColor(img_rgb, cv.COLOR_BGR2HSV)
        mask = cv.inRange(img_hsv, lower, upper)
        mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
        mask = cv.morphologyEx(mask, cv.MORPH_DILATE, kernel)
        
        contours, hierarchy = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
        if len(contours) != 0:
                c = max(contours, key = cv.contourArea)
                if cv.contourArea(c)>5:
                    M = cv.moments(c)
                    cX = int(round(M["m10"] / M["m00"]))
                    cY = int(round(M["m01"] / M["m00"]))
                    rect = cv.minAreaRect(c)
                    box = cv.boxPoints(rect)
                    box = np.int64(box)
                    return box,mask,[cX,cY]
                else:
                    return None,mask ,None

        else:
            return None,mask ,None
        
    def find_point2(self,img_rgb,lower1,upper1,lower2,upper2,kernel):
        img_hsv = cv.cvtColor(img_rgb, cv.COLOR_BGR2HSV)
        mask0 = cv.inRange(img_hsv, lower1, upper1)
        mask1 = cv.inRange(img_hsv, lower2, upper2)
        mask = mask0+mask1
        mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
        mask = cv.dilate(mask,kernel,iterations = 2)
        
        contours, hierarchy = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
        if len(contours) != 0:
                c = max(contours, key = cv.contourArea)
                M = cv.moments(c)
                cX = M["m10"] / M["m00"]
                cY = M["m01"] / M["m00"]
                cv.circle(img_rgb,(int(round(cX)),int(round(cY))),1,(0,0,255),-1)
                return [cX,cY]
        else:
            return None
        pass


    def rescale_frame(self,frame, percent=75):
        width = int(frame.shape[1] * percent/ 100)
        height = int(frame.shape[0] * percent/ 100)
        dim = (width, height)
        return cv.resize(frame, dim, interpolation =cv.INTER_AREA)

def main(args):
    rospy.init_node('sphero_detection', anonymous=True)
    detection_hsv = sphero_detection_hsv()
    rate = rospy.Rate(10)

    try:
        while not rospy.is_shutdown():
            rate.sleep()
    except KeyboardInterrupt:
        print("Shutting down")
        cv.destroyAllWindows()

        
if __name__ == '__main__':
    main(sys.argv)
