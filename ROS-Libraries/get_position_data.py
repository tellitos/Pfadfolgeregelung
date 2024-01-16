#!/usr/bin/env python3

import rospy
import math

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
import time
import cv2 as cv
import numpy as np
import sys
import time
import matplotlib.pyplot as plt


''' 
    - get Sphero position and target velocity 
    - calc velocity 
    - show results and save @/home/mas/SAM_V3 
----
Params:
    - pixel_to_point_ratio = 2.5/1000   1 pixel := 2.5mm

----
Publisher:
    - 

Subcriber:
    - /sphero_position_green  PoseStamped 
    - /sphero_position_green_mean  PoseStamped 
    - //sphero_0/cmd_vel  Twist 

'''


class get_data():

    def __init__(self):
        self.twist = Twist()
        self.stop = True
        self.position = np.empty((0,3), float)
        self.position_interp = np.empty((0,3), float)
        self.position_mean = np.empty((0,3), float)
        self.cmd_vel = np.empty((0,3), float)
        
        self.image_sub = rospy.Subscriber('/sphero_position_green',PoseStamped,self.callback_position,queue_size=1)
        #self.image_sub_interp = rospy.Subscriber('/sphero_position_green_mean',PoseStamped,self.callback_position_mean,queue_size=1)
        #self.image_sub_interp = rospy.Subscriber('/sphero_position_green',PoseStamped,self.callback_position_mean,queue_size=1)
        #self.cmd_vel_sub = rospy.Subscriber('/sphero_0/cmd_vel',Twist,self.callback_cmd_vel,queue_size=1)
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel',Twist,self.callback_cmd_vel,queue_size=1)


    def callback_position(self,pose):
        if self.stop == False:
            t =  float(pose.header.stamp.to_sec())
            self.position = np.append(self.position,np.array([[pose.pose.position.x,pose.pose.position.y,t]]),axis = 0)
        pass

    def callback_position_mean(self,pose):
        if self.stop == False:
            t3 =  float(pose.header.stamp.to_sec())
            self.position_mean= np.append(self.position_mean,np.array([[pose.pose.position.x,pose.pose.position.y,t3]]),axis = 0)
        pass

    def callback_cmd_vel(self,twist):

        if self.stop == False:
            t2 =  float(rospy.Time.from_sec(time.time()).to_sec())
            self.cmd_vel = np.append(self.cmd_vel,np.array([[twist.linear.x,twist.angular.z,t2]]),axis = 0)
        pass

    def normalize_angle_positive(self, angle):
        """Return positive angle in radians. [0,2pi]"""
        return math.fmod(math.fmod(angle, 2.0 * math.pi) + 2.0 * math.pi, 2.0 * math.pi)
    

if __name__ == '__main__':

    rospy.init_node('stepresponse', anonymous=True)
    data = get_data()
    pixel_to_point_ratio = 2.5 / 1000

    starting_time = float(rospy.Time.from_sec(time.time()).to_sec())
    data.stop = False
    while not rospy.is_shutdown():
        rospy.sleep(0.01)

    data.stop = True
    stop_time = float(rospy.Time.from_sec(time.time()).to_sec())
    length = int(math.ceil(stop_time-starting_time ))

    times_cmd = data.cmd_vel[:,2]-starting_time
    x_cmd = data.cmd_vel[:,0]
    z_cmd = data.cmd_vel[:,1]

    times = data.position[:,2]-starting_time
    x = data.position[:,0]
    y = data.position[:,1]
    #dx= np.gradient(x,times)
    #dy = np.gradient(y,times)
    #vel= np.sqrt(np.square(dx)  + np.square(dy))


    x_mean = data.position_mean[:,0]
    y_mean = data.position_mean[:,1]
    times_mean= data.position_mean[:,2]-starting_time
    dx_mean = np.gradient(x_mean,times_mean)
    dy_mean = np.gradient(y_mean,times_mean)
    vel_mean= (np.sqrt(np.square(dx_mean)  + np.square(dy_mean))*pixel_to_point_ratio )*1000
    diff_x = np.diff(x_mean,prepend=x_mean[0])
    diff_time = np.diff(times_mean,prepend=times_mean[0])
    
    orientation = []
    for i in range(0,len(times_mean)):
        if vel_mean[i]<40:
            cmd_heading = 0
        else:
            cmd_heading = int(data.normalize_angle_positive(
                math.atan2(dy_mean[i],dx_mean[i],)) * 180 / math.pi)
            cmd_heading = (cmd_heading + 180) % 360 - 180
        orientation.append([cmd_heading])
    

    fig, (ax1, ax2,ax3,ax4) = plt.subplots(4)
    ax1.plot(times_cmd,x_cmd,'b-')
    ax1.plot(times_cmd,z_cmd,'r-')
    ax1.set_xticks(np.arange(0,length,1))
    ax1.set_xlabel('time in sec')
    ax1.set_ylabel('Stephight/angle')
    ax1.grid()


    ax2.plot(times_mean, x_mean , 'rx')
    ax2.plot(times_mean, y_mean , 'b-')
    ax2.set_xticks(np.arange(0,length,1))
    ax2.set_xlabel('time in sec')
    ax2.set_ylabel('Position in Pixel_mean')
    ax2.grid()

    ax3.plot(times, x , 'r-')
    ax3.plot(times, y , 'b-')
    ax3.set_xticks(np.arange(0,length,1))
    ax3.set_xlabel('time in sec')
    ax3.set_ylabel('Position in Pixel')
    ax3.grid()

    ax4.plot(times_mean, diff_time , 'ro')
    ax4.set_xticks(np.arange(0,length,1))
    ax4.set_xlabel('time in sec')
    ax4.set_ylabel('cycle time in sec')
    ax4.grid()

    fig, (ax1, ax2,ax3,ax4) = plt.subplots(4)
    if len(x_cmd) >0:
        ax1.stem(times_cmd,x_cmd)
        ax1.stem(times_cmd,z_cmd,'r')
    ax1.set_xticks(np.arange(0,length,0.25))
    ax1.set_xlabel('time in sec')
    ax1.set_ylabel('Stephight')
    ax1.grid()

    x_label = ax2.plot(times_mean, x_mean , 'r-x',label='x')
    y_label= ax2.plot(times_mean, y_mean , 'b-',label='y')
    ax2.set_xticks(np.arange(0,length,1))
    ax2.set_xlabel('time in sec')
    ax2.set_ylabel('Position in Pixel')
    ax2.legend()
    ax2.grid()

    ax3.plot(times_mean, vel_mean , 'g-')
    ax3.set_xticks(np.arange(0,length,0.25))
    ax3.set_xlabel('time in sec')
    ax3.set_ylabel('Velocity in mm/s interp')
    ax3.grid()

    ax4.plot(times_mean, orientation , 'g-')
    ax4.set_xticks(np.arange(0,length,1))
    ax4.set_xlabel('time in sec')
    ax4.set_ylabel('Orientation')
    ax4.grid() 

    fig, (ax1) = plt.subplots(1)
    ax1.scatter(x_mean, y_mean)
    ax1.set_xlabel('x')
    ax1.set_ylabel('y')
    ax1.set_xticks(np.arange(0,1277,250))
    ax1.set_yticks(np.arange(0,1277,250))
    ax1.invert_yaxis()
    timestr = time.strftime("%Y%m%d-%H%M%S")
    input_file = "/home/mas/saved_data/input_"+timestr+".csv"
    output_file = "/home/mas/saved_data/output_"+timestr+".csv"
    #with open("/home/mas/SAM_V3/input.csv", "w") as m:
    with open(input_file, "w") as m:
        m.write("time,input_v,input_theta\n")
        for i in range(0,len(times_cmd)):
            m.write(f"{times_cmd[i]},{x_cmd[i]},{z_cmd[i]}\n")
           # print(times_cmd[i])

    with open(output_file, "w") as f:
        f.write("time,x,y,vel,orient\n")
        for n in range(0,len(times_mean)):
            f.write(f"{times_mean[n]},{x_mean[n]},{y_mean[n]},{vel_mean[n]},{orientation[n]}\n")

    plt.show()
    time.sleep(1)
    sys.exit()



 