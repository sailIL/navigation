#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

class TwistSubscriber:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('twist_subscriber')

        # Create subscriber for "filter/twist" topic
        rospy.Subscriber('filter/twist', Twist, self.twist_callback)

        # Create publisher for "cmd_vel" topic
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    def twist_callback(self, msg):
        # Use values from "filter/twist" message to create new "cmd_vel" message
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.y = 2
        cmd_vel_msg.linear.x =  - msg.angular.z * 2

        # Publish new "cmd_vel" message
        self.cmd_vel_pub.publish(cmd_vel_msg)

if __name__ == '__main__':
    # Create instance of TwistSubscriber class
    twist_subscriber = TwistSubscriber()

    # Spin the node to keep it running
    rospy.spin()
