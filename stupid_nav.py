#!/usr/bin/env python2.7

import math
import cmd
from unittest import case
import rospy
import math
import actionlib
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Twist, PoseStamped
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_euler 
from std_msgs.msg import Int32, Float64, String

# the state machine is: 
# START 
# NO_BOUY0 -starting the mission and we don't see any bouy in the ball _array 
# ONE_BOUY_OR_MORE0 - there is one bouys at least in the marker array  and we didn't pass yet the fisrst gate
# CLOSE_TO_THE_BOUYS0 - we are 2 meter before the first gate, we will continue stragiht for 3 sec
# NO_BOUY1 -  we don't see any bouy in the ball _array , but we saw one already (we will contionue straight in x)
# ONE_BOUY_OR_MORE1 - there is one bouys at least in the marker array  and we  passed  the first gate
# CLOSE_TO_THE_BOUYS1 - we are 2 meter before the second gate, we will continue stragiht for 3 sec
# FINISH
POLE = 3
DIST_TO_MARKER_TH = 1.5
SEC_TO_BYPASS_BUOYS = 7 #5 
SEC_TO_WAIT =2
TRUE = 1
CLOSE_TO_BOUYS = 1
NOT_CLOSE_TO_BOUYS =0

START = 0
NO_BOUY0 = 1
ONE_BOUY_OR_MORE0 = 2
CLOSE_TO_THE_BOUYS0 = 3
NO_BOUY1 = 4
ONE_BOUY_OR_MORE1 = 5  
CLOSE_TO_THE_BOUYS1 = 6
FINISH = 7

class RelativeNav:
    def __init__(self):
        #NEED TO CHECK WHAT TO PUT IS SELF
        #note: error distance is the same like the goal
        self.goal_x = -1
        self.goal_y = -1
        self.goal_z = 0
        self.goal_yaw = -1
        self.state = START
        self.activated = TRUE
        #####
        self.num_of_markers = -1
        rospy.Subscriber("/ball_marker", MarkerArray, self.marker_array_callback)
        self.cmd_vel_pub=rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        ## system debug
        self.status_pub = rospy.Publisher("/status", Int32, queue_size=10)
        self.test = rospy.Publisher("/test", Int32, queue_size=10)
        ## rviz debug
        self.rviz_pub=rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)

    def marker_array_callback(self,marker_array):
        closest_green_marker = Marker()
        closest_green_marker.pose.position.x = -1
        closest_red_marker = Marker()
        closest_red_marker.pose.position.x = -1
        green_pole_flag = 0 # if there was at least 1 green pole -> green_pole_flag = 1
        red_pole_flag = 0 # if there was at least 1 red pole -> red_pole_flag = 1
        self.num_of_markers = 0
        for i in range(len(marker_array.markers)):
            if  not (math.isnan(marker_array.markers[i].pose.position.x)):  # if the marker[i] position is not none => there is something there
                marker_array.markers[i].pose.position.y = -1* marker_array.markers[i].pose.position.y   # why?!
                if (marker_array.markers[i].type == POLE and marker_array.markers[i].color.g == 1.0 ): # we want to find  a green pole
                    green_pole_flag = 1
                    if (closest_green_marker.pose.position.x == -1 or marker_array.markers[i].pose.position.x < closest_green_marker.pose.position.x):  #finding the closest green pole based on x value.
                        closest_green_marker = marker_array.markers[i]
                elif (marker_array.markers[i].type == POLE and marker_array.markers[i].color.r == 1.0): # we want to find a red pole
                    red_pole_flag = 1
                    if (closest_red_marker.pose.position.x == -1 or marker_array.markers[i].pose.position.x < closest_red_marker.pose.position.x):
                        closest_red_marker = marker_array.markers[i]
        print("red_pole_flag = " + str(red_pole_flag) + "green_pole_flag = " + str(green_pole_flag) )
        if (red_pole_flag == 1 and green_pole_flag == 1):
            self.goal_x = (closest_green_marker.pose.position.x + closest_red_marker.pose.position.x)/2 #- 3
            #print(closest_green_marker.pose.position.y, '+', closest_red_marker.pose.position.y, '=', closest_green_marker.pose.position.y+closest_red_marker.pose.position.y)
            self.goal_y = ( closest_green_marker.pose.position.y + closest_red_marker.pose.position.y)/2 #
            self.num_of_markers = 2
        elif (green_pole_flag == 1):   # you always need to place the green pole on your right
            #print('Red pole flag = ', red_pole_flag)
            #print('Green y position = ', closest_green_marker.pose.position.y)
            self.goal_x = closest_green_marker.pose.position.x #- 3
            self.goal_y = closest_green_marker.pose.position.y -2 #TODO checking if the green on the right or on the left
            self.num_of_markers =1
        elif (red_pole_flag == 1):
            #print('Red y position = ', closest_red_marker.pose.position.y)
            self.goal_x = closest_red_marker.pose.position.x #- 3
            self.goal_y = closest_red_marker.pose.position.y +2 #TODO
            self.num_of_markers =1
        print ("cv fault ? : goal x" + str(self.goal_x) + "goaly: " + str(self.goal_y))

    def pid(self, error, gain):
        # control expect velocity between 1 to -1. we want to go slow so we chose 0.5
        cmd =gain*error
        # if (cmd > 0.1):
        #     cmd= 0.1
        # if (cmd< -0.1):
        #    cmd =-0.1
        return cmd

    def control_velocity(self, close_to_the_bouys =0):
        if (close_to_the_bouys): # we don't want to slow down when we are near to the bouys
            errx =self.goal_x +3
            erry =self.goal_y 
            print("DEBUG: reached close_to_buoys, errx = " + str(errx) + "erry = " + str(erry))
        else:    
            errx =self.goal_x
            erry =self.goal_y
        errz = 0
        erryaw= math.atan(erry/errx)
        print( "debug: errx: " + str(errx)+"erry: " + str(erry)+"erryaw: " + str(erryaw))
        #normalize the values to 1 
        #max_value =max([erry,errx])
        #norm_errx = errx /max_value
        #norm_erry = erry / max_value
        #cmd_vel_x=self.pid(errx, 0.5) # the max value of the velocity will be 0.1
        #cmd_vel_y=self.pid(erry, 0.5)
        #cmd_vel_yaw=self.pid(erryaw, 0.01)
        #cmd_vel_yaw=0.0
        print("cmd_vel: cmd_vel_x = " + str(errx) + "cmd_vel_y = " + str(erry)  )
        #publish the message 
        cmd_vel_msg=Twist()
        cmd_vel_msg.linear.x=errx
        cmd_vel_msg.linear.y=erry
        cmd_vel_msg.linear.z = 0
        cmd_vel_msg.angular.z=0
        # if (self.goal_x <= 0): #arrived to the green pole
        #     print("zro msg")
        #     cmd_vel_msg=Twist()
        #     cmd_vel_msg.linear.x=0
        #     cmd_vel_msg.linear.y=0
        #     cmd_vel_msg.linear.z = 0
        #     cmd_vel_msg.angular.z=0
        self.cmd_vel_pub.publish(cmd_vel_msg)
        ###DEBUG############
        rviz_msg = PoseStamped()
        rviz_msg.pose.position.x = errx
        rviz_msg.pose.position.y = erry
        rviz_msg.pose.position.z = 0#errz
        q_err = quaternion_from_euler(0,0, erryaw)
        rviz_msg.pose.orientation.x = q_err[0]
        rviz_msg.pose.orientation.y = q_err[1]
        rviz_msg.pose.orientation.z = q_err[2]
        rviz_msg.pose.orientation.w = q_err[3]
        rviz_msg.header.frame_id = 'world'
        self.rviz_pub.publish(rviz_msg)

    def straight(self):
        cmd_vel_msg=Twist()
        cmd_vel_msg.linear.x= 2
        cmd_vel_msg.linear.y= 0
        cmd_vel_msg.linear.z = 0
        cmd_vel_msg.angular.z=0
        self.cmd_vel_pub.publish(cmd_vel_msg)
        ###DEBUG############
        rviz_msg = PoseStamped()
        rviz_msg.pose.position.x = 1
        rviz_msg.pose.position.y = 0
        rviz_msg.pose.position.z = 0#errz
        rviz_msg.header.frame_id = 'world'
        self.rviz_pub.publish(rviz_msg)
    
    def stop (self):
        cmd_vel_msg=Twist()
        cmd_vel_msg.linear.x= 0
        cmd_vel_msg.linear.y= 0
        cmd_vel_msg.linear.z = 0
        cmd_vel_msg.angular.z=0
        self.cmd_vel_pub.publish(cmd_vel_msg)



def main():
    rospy.init_node("stupid_nav", anonymous= False)
    rate= rospy.Rate(2)
    rospy.sleep(1)
    R= RelativeNav()

    
    while not rospy.is_shutdown() and R.activated:

        if (R.state == START):
            R.test.publish(R.state)
            if R.goal_x > 0:
                R.state = ONE_BOUY_OR_MORE0
            else:
                R.state = NO_BOUY0
        
        if (R.state == NO_BOUY0):
            R.test.publish(R.state)
            initTime = rospy.Time.now().secs
            while not rospy.is_shutdown(): 
                R.straight()
                if rospy.Time.now().secs - initTime >SEC_TO_WAIT:
                        R.state = START
                        break
                rate.sleep()
        
        if (R.state == ONE_BOUY_OR_MORE0):
            R.test.publish(R.state)
            if R.goal_x <DIST_TO_MARKER_TH:
                print ("thershold" +str(R.goal_x))
                R.state = CLOSE_TO_THE_BOUYS0
            else:
                R.control_velocity()
                ##print ("ONE_BOUY_OR_more goal x " +str(R.goal_x) +"goal y"+str(R.goal_y))
        
        if (R.state == CLOSE_TO_THE_BOUYS0):
            R.test.publish(R.state)
            initTime = rospy.Time.now().secs
            while not rospy.is_shutdown(): ## TODO change the timner 
                print ("timer0" + str(R.goal_x))
                R.control_velocity(CLOSE_TO_BOUYS) ## we want not to slow down near the bouys
                if rospy.Time.now().secs - initTime >SEC_TO_BYPASS_BUOYS:
                        #print ("timer0" + str(R.goal_x))
                        #if R.num_of_markers >0:
                        #   R.state = ONE_BOUY_OR_MORE1
                        #else: 
                        #    R.state = NO_BOUY1
                        #break
                        print("finish")
                        R.state = FINISH
                        break
                rate.sleep()
        #try
        #if (R.state == NO_BOUY1):  
        #    R.test.publish(R.state)
        #    initTime = rospy.Time.now().secs
        #    while not rospy.is_shutdown(): ## TODO change the timner 
        #        R.straight()
        #        if rospy.Time.now().secs - initTime >SEC_TO_WAIT:
        #                if R.num_of_markers >0:
        #                   R.state = ONE_BOUY_OR_MORE1
        #                else: 
        #                    R.state = NO_BOUY1
        #                break
        #        rate.sleep()     
#
        #if (R.state == ONE_BOUY_OR_MORE1):
        #    R.test.publish(R.state)
        #    if R.goal_x <DIST_TO_MARKER_TH:
        #        R.state = CLOSE_TO_THE_BOUYS1
        #        print ("thershold2 " + str(R.goal_x))
        #    else:
        #        R.control_velocity()
        #        print ("ONE_BOUY_OR_more"+ str(R.goal_x))
		#print("ONE_BOUY_OR_more - 1 (passed gate 1)")
        #
        #if (R.state == CLOSE_TO_THE_BOUYS1):
        #    R.test.publish(R.state)
        #    initTime = rospy.Time.now().secs
        #    while not rospy.is_shutdown(): ## TODO change the timner 
        #        R.control_velocity() # we want to slow down near the bouys - second gate
        #        if rospy.Time.now().secs - initTime >SEC_TO_BYPASS_BUOYS: 
        #                print("finish")
        #                R.state = FINISH
        #                break
        #        rate.sleep()
        
        if (R.state == FINISH):
            R.test.publish(R.state)
            R.stop()
            R.activated = False
            R.status_pub.publish(1)
                        
        rate.sleep()
    rospy.spin()

if __name__== "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass # to check what it means
