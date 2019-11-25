#!/usr/bin/env python

###################  import  ###########################
import rospy
from sensor_msgs.msg import Joy     #joystick message type
from geometry_msgs.msg import Twist #turtlebot message type

############# instantiation of objects  #####################
distance=Twist()

#############  node initialization  ####################
rospy.init_node('teleop', anonymous=True)

##### initialize values ####
distance.linear.x = 0
distance.angular.z = 0

############ definitions of functions ##################
def callback(data):
    global distance                #message object
    distance.linear.x=data.axes[1]
    distance.angular.z=20*data.axes[3]

#### definition of publisher/subscriber and services ###
rospy.Subscriber("joy", Joy, callback)                  #Subscriber joystick
pub = rospy.Publisher('robot/cmd_vel', Twist, queue_size=10)  #Publisher velocities

############# main program #############################
rate = rospy.Rate(10)

#--------------endless loop till shut down -------------#
while not rospy.is_shutdown():
    pub.publish(distance)       #publish velocity commands
    rate.sleep()
