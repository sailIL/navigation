#!/usr/bin/env python2.7

import math
import rospy
import math
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
import time

BOUY = 2 #marker array sphere is 2

class Obsatcle_Chan:
    def __init__(self):
        self.objects_list = []
        self.blue_ball_x_pos = 0
        self.blue_ball_y_pos = 0
        self.found_blue_ball = 0
        self.current_pose_x = 0 
        self.current_pose_x = 0
        self.init_odom_pose_x = 0 #the init position from which we go to the our target based on odometry send command
        self.init_odom_pose_y = 0 #the init position from which we go to the our target based on odometry send command

        self.reached_the_ball = 0

        self.decision_distance = 2 # the distance from the ball in which we move to odometry based commands
        self.odometry_radius_error = 1 # the radius error from the wanted posiiton in which we decide that we have reached the position.

        # ROS Subscribers
        rospy.Subscriber("/ball_marker", MarkerArray, self.marker_array_callback)
        rospy.Subscriber("odometry", MarkerArray, self.odometry_callback)
        self.cmd_vel_pub=rospy.Publisher("/cmd_vel", Twist, queue_size=10)


    def marker_array_callback (self,marker_array):
        self.objects_list = [] # in this callback , we are taking only the bouys without the poles
        for i in range(len(marker_array.markers)):
            if (marker_array.markers[i].type == BOUY) and  not (math.isnan(marker_array.markers[i].pose.position.x)):
                marker_array.markers[i].pose.position.y = -1* marker_array.markers[i].pose.position.y   # our y axis is opposite (need to change in the future)
                self.objects_list.append(marker_array.markers[i])

    def odometry_callback(self,msg):
        self.current_pose_x = msg.pose.pose.position.x
        self.current_pose_y = msg.pose.pose.position.y

    def compute_goal(self, objects_list):
        # first we want to find the closet green , yellow , red bouys
        closest_blue_marker = Marker()
        closest_blue_marker.pose.position.x = -1

        for i in range(len(objects_list)):
            if (objects_list[i].color.g == 0 and  objects_list[i].color.r == 0 and  objects_list[i].color.b == 1.0): # blue 
                print("found blue ball")
                self.found_blue_ball = 1
                # finding the position of the ball
                self.blue_ball_x_pos = objects_list[i].pose.position.x
                self.blue_ball_y_pos = objects_list[i].pose.position.y

    def calculate_go_to_position_from_cv(self):
        # we need to circle the ball - arbitrary desicion: circle CW
        go_to_x_position = self.blue_ball_x_pos
        go_to_y_position = self.blue_ball_y_pos - 2  # go 2 meters to the right from the blue ball
        msg = Twist()
        msg.linear.x = go_to_x_position
        msg.linear.y = go_to_y_position
        msg.angular.z = 0
        self.cmd_vel_pub.publish(msg)

    def calculate_go_to_position_from_odometry(self):

        #
        go_to_odom_pose_x = self.current_pose_x + 8 
        go_to_odom_pose_y = self.current_pose_y - 2
        msg = Twist()
        msg.linear.x = go_to_odom_pose_x
        msg.linear.y = go_to_odom_pose_y
        msg.angular.z = 0
        self.cmd_vel_pub.publish(msg)
        time.sleep(3)
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.angular.z = math.radians(180)
        self.cmd_vel_pub.publish(msg)
        time.sleep(3)

    def distance_between_points(self,x1, y1, x2, y2):
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def situation_machine(self): #decides if move commands are based on odometry (when we cant see the cannon => when we are next to it) or from input from cv
        if self.reached_the_ball == 0:
            if self.blue_ball_x_pos <= self.decision_distance:
                self.reached_the_ball = 1 
                self.calculate_go_to_position_from_odometry()
            else:
                self.calculate_go_to_position_from_cv()
        if self.reached_the_ball == 1:
            pass

            



