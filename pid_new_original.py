#!/usr/bin/env python

import rospy
from simple_pid import PID
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from sensor_msgs.msg import Joy


import csv
import time
import tf

pid_x = PID(0.2, 0, 0.1, setpoint = 0)
pid_y = PID(0.2, 0, 0.1, setpoint = 0)


opti_x = 0
opti_y = 0
opti_z = 0
point = 0


pre_x = 0
pre_y = 0

next_ = 0

yaw_degrees = 0


def heading_comp(actual, waypoint):
    difference = waypoint - actual
    while difference < -180:
        difference += 360
    while difference > 180:
        difference -= 360


def odom_callback(msg):
    global point, opti_x, opti_y, opti_z, yaw_degrees

    pub_bebop_vel = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=1)
    twist_bebop = Twist()


    opti_x = msg.pose.pose.position.y
    opti_y = msg.pose.pose.position.x
    opti_z = msg.pose.pose.position.z

    opti_ang_x = msg.pose.pose.orientation.x
    opti_ang_y = msg.pose.pose.orientation.y
    opti_ang_z = msg.pose.pose.orientation.z
    opti_ang_w = msg.pose.pose.orientation.w
    
    quaternion = [opti_ang_x, opti_ang_y, opti_ang_z, opti_ang_w]

    euler = tf.transformations.euler_from_quaternion(quaternion)
    yaw = euler[2]
    
    


    x = pid_x(opti_x)
    y = pid_y(opti_y)

    error_x = pid_x.setpoint - opti_x
    error_y = pid_y.setpoint - opti_y

    print("opti_x ==> " + str(opti_x))
    print("opti_y ==> " + str(opti_y))

    #print("error x ==> " + str(error_x))
    #print("error y ==> " + str(error_y))

    #print("control signal x ==> " + str(x))
    #print("control signal y ==> " + str(y))
    #print("==============================")
    twist_bebop.linear.x = x
    twist_bebop.linear.y = -y
    pub_bebop_vel.publish(twist_bebop)
    
    
    if abs(error_x) < 0.1 and abs(error_y) < 0.1 and point == 0:
        pid_x.setpoint = 0.7
        pid_y.setpoint = -1.5
        point += 1
    elif abs(error_x) < 0.1 and abs(error_y) < 0.1 and point == 1:
        pid_x.setpoint = 0.6
        pid_y.setpoint = 0.9
        point += 1
    elif abs(error_x) < 0.1 and abs(error_y) < 0.1 and point == 2:
        pid_x.setpoint = -0.9
        pid_y.setpoint = 0.7
        point += 1
    elif abs(error_x) < 0.1 and abs(error_y) < 0.1 and point == 3:
        pid_x.setpoint = -0.9
        pid_y.setpoint = -0.7
        point += 1
    elif abs(error_x) < 0.1 and abs(error_y) < 0.1 and point == 4:
        pid_x.setpoint = 0
        pid_y.setpoint = 0
    print(str(point)) 


def main():
    # Initialize node
    rospy.init_node('logger', anonymous=True)

    # Subscribing to the mocap_node and the Odometry topic
    rospy.Subscriber('/mocap_node/bebop_x/Odom', Odometry, odom_callback)
    pub_land = rospy.Publisher('/bebop/land', Empty, queue_size=10)
    pub_takeoff = rospy.Publisher('/bebop/takeoff', Empty, queue_size = 10)

    rospy.spin()


if __name__ == '__main__':
    starting = input("Press s to start: ")
    if starting == "s":
        print("Starting main()")
        main()
        print("Done!!")

