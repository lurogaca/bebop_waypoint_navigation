import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from simple_pid import PID

import time


#publishers to takeoff and land drone
pub_takeoff = rospy.Publisher(f'/bebop_0x/takeoff', Empty, queue_size=5)
pub_land = rospy.Publisher(f'/bebop_0x/land', Empty, queue_size=5)



def callback():
    #put procedure here
    pass


def main():
    # Initialize node
    rospy.init_node('logger', anonymous=True)
   
    
    pub_takeoff.publish(Empty())
    print('takeoff')
    time.sleep(5)

    # Subscribing to the mocap_node and the Odometry topic
    rospy.Subscriber('/mocap_node/bebop_0x/Odom', Odometry, callback)

    
    input('Press enter to stop and land drone')

    pub_land.publish(Empty())


if __name__ == '__main__':
    print("Starting main()")

    rospy.init_node('myNode')
    time.sleep(2)
    main()

    print("Done!!")