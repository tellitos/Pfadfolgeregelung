#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from math import atan2,sqrt,exp
import time

weg=[(833,1137),(374,899),(717,866),(960,682),(247,606),(422,440),(388,120),(210,127)]
weg2=(374,899)
weg3=(792,963)
weg4=(1065,760)
weg5=(300,677)
weg6=(488,511)
weg7=(467,162)
weg8=(279,173)
i=1

global pub

def listener():
    objektiv=1
    rospy.init_node('ros_package', anonymous=True)
    rospy.Subscriber('/turtlebot_position', PoseStamped, callback) 
    rospy.spin()

def callback(data):
    global i
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    twist = Twist()
    weg2=weg[i]
    print("X: "+str(data.pose.position.x))
    print("Y: "+str(data.pose.position.y))
    print("Winkel ist: "+str(data.pose.orientation.z))
    print("Objektiv ist: "+str(weg2))
    if data.pose.position.x==-1.0:
        print("Not detected")
        #twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        #twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
    else:
        current_x=data.pose.position.x
        current_y=data.pose.position.y
        angIst=data.pose.orientation.z
        if abs(current_x-weg2[0]<2) and abs(current_y-weg2[1])<2:
            if (i<7):
                i=i+1
            elif (i==7):
                i=0
        angSoll=atan2(weg2[1]-current_y,weg2[0]-current_x)
        print("Winkel soll: ",angSoll)
        if angIst<0 and angSoll<0:
                ang3=angIst-angSoll
        elif angIst>0 and angSoll>0:
               ang3=angIst-angSoll
        else:
            ang3=angIst+angSoll
        if ((ang3<0.02 and ang3>0)and (angIst>0 and angSoll>0)) or ((ang3>-0.02 and ang3<0) and (angIst<0 and angSoll<0)):
            for d in range(0,2):
                if abs(current_x-weg2[0]<2) and abs(current_y-weg2[1])<2:
                    break
                dist=sqrt((weg2[0]-current_x)**2+(weg2[1]-current_y)**2)
                dist=dist*1
                control_linear_vel = dist
                print("Geschwindigkeit: ", dist)
                twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0
                twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
                pub.publish(twist)
                time.sleep(0.2)
        elif (angIst<0 and angSoll>0):
            twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.25
            pub.publish(twist)
        elif (angIst>0 and angSoll<0):
            twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = -0.25
            pub.publish(twist)
        else:
            twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = ang3
            pub.publish(twist)
    

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
