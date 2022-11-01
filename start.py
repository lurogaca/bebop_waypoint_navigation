import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from simple_pid import PID

import time
import sys
import math
import tf


#publishers to takeoff and land drone
pub_takeoff = rospy.Publisher(f'/bebop_0x/takeoff', Empty, queue_size=5)
pub_land = rospy.Publisher(f'/bebop_0x/land', Empty, queue_size=5)
velPub = rospy.Publisher(f'/bebop_0x/cmd_vel', Twist, queue_size=1)


#running procedure for Drone
def callback(msg):
    # a quaternion is a 4 tuple that determines the orientation of an object.
    quaternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    
    #we convert the quaternion into traditional yaaw, pitch, and roll rotation axis
    # NOTE: These measurements are in Radians and not degrees
    euler = tf.transformations.euler_from_quaternion(quaternion)
    
    # we get the yaw from the euler measurements
    # subtract 90 degrees to align the drone axis and room axis
    yaw_rad = euler[2] - (math.pi/2)

    #after the subtraction, we re align the values to be in range -180 to 180 degrees
    if (yaw_rad < -(math.pi)):
        yaw_rad += (2 * math.pi)
    
    # position variable that holds the x, y position as well as the yaw in radians of the drone.
    pos = [msg.pose.pose.position.x, msg.pose.pose.position.y, yaw_rad]

    # Create Twist object to publish to drone
    myTwist = Twist()

    #example of how to set the velocities of the drone on each axis
    myTwist.linear.x = 0
    myTwist.linear.y = 0
    myTwist.linear.z = 0
    # Angular z is the heading of the drone
    myTwist.angular.z = 0

    #publish our Twist object to the Deones velocity topic
    #NOTE: This will make the drone move if connection is secured
    velPub.publish(myTwist)

def main():

    #force input from user to takeoff drones
    myIn = input('press t to takeoff: ')

    #if not what is expected then exit the program
    if myIn != 't':
        print('Invalid input')
        sys.exit(1)
    
    
    pub_takeoff.publish(Empty())
    print('takeoff')
    
    time.sleep(3)

    # Subscribing to the mocap_node and the Odometry topic
    #NOTE: This starts the procedure in callback
    rospy.Subscriber('/mocap_node/bebop_0x/Odom', Odometry, callback)

    
    # wait for user input to land the drone and end program
    input('Press enter to stop and land drone')

    pub_land.publish(Empty())

    time.sleep(3)


if __name__ == '__main__':
    print("Starting main()")

    #initialize node for ros
    rospy.init_node('myNode')
    
    #wait for publishers to form
    time.sleep(2)

    #run main
    main()

    print("Done!!")