#!/usr/bin/env python2.7

import math
import time
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

#INFO: OBSTACLE CHANEL USING THR CV

'''
method: 
    - wait until we have at least 2 bouys (we assume that we are in the front o f the gate)
    - we will go to the goal that is changing all the time by the marker_array, until we don't any marker in the array
    
'''
BOUY = 2 #marker array sphere is 2
FIRST_BLUE = False
PAST_BLUE = False
END = False
DIS = 4
class Obsatcle_Chan:
    def __init__(self):
        self.activated = True
        self.state = -1

        self.objects_list = []

        self.goal_x = -1
        self.goal_y = -1
        self.goal_yaw = -1


        # ROS Subscribers
        rospy.Subscriber("/ball_marker", MarkerArray, self.marker_array_callback)
        self.cmd_vel_pub=rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        ## system debug
        self.status_pub = rospy.Publisher("/status", Int32, queue_size=10)
        self.test = rospy.Publisher("/test", Int32, queue_size=10)
        self.rviz_pub=rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)


    def marker_array_callback (self,marker_array):
        self.objects_list = [] # in this callback , we are taking only the bouys without the poles
        for i in range(len(marker_array.markers)):
                if (marker_array.markers[i].type == BOUY) and  not (math.isnan(marker_array.markers[i].pose.position.x)):
                    marker_array.markers[i].pose.position.y = -1* marker_array.markers[i].pose.position.y
                    self.objects_list.append(marker_array.markers[i])

    def compute_goal(self, objects_list):
        # first we want to find the closet green ,red bouys
            closest_green_marker = Marker()
            closest_green_marker.pose.position.x = -1
            closest_red_marker = Marker()
            closest_red_marker.pose.position.x = -1
            green_bouy_flag = 0 # if there was at least 1 green bouy -> green__flag = 1
            red_bouy_flag = 0 # if there was at least 1 red bouy -> red__flag = 1
            #yellow_bouy_flag = 0 # if there was at least 1 yellow bouy -> yellow__flag = 1
        
            for i in range(len(objects_list)):
                if (objects_list[i].color.g == 1.0 and  objects_list[i].color.r == 0 ): # we want to find  a green bouy
                    green_bouy_flag = 1
                    if (closest_green_marker.pose.position.x == -1 or objects_list[i].pose.position.x < closest_green_marker.pose.position.x):
                        closest_green_marker = objects_list[i]
                elif (objects_list[i].color.r == 1.0 and objects_list[i].color.g == 0 ): # we want to find a red bouy
                    red_bouy_flag = 1
                    if (closest_red_marker.pose.position.x == -1 or objects_list[i].pose.position.x < closest_red_marker.pose.position.x):
                        closest_red_marker = objects_list[i]
                elif (objects_list[i].color.b == 1.0 and objects_list[i].color.g == 0 and objects_list[i].color.r == 0  ): #we want to find a blue bouy
                    blue_bouy_flag = 1
                    blue_marker = objects_list[i]
            
            if(blue_bouy_flag):
                if (FIRST_BLUE):
                    print("I saw blue bouy")
                    self.goal_x = blue_marker.pose.position.x
                    self.goal_y = blue_marker.pose.position.y 
                    FIRST_BLUE = True 
                    #timer = time.start  # TODO not sure on the sentace
                else:
                    if(blue_marker.pose.position.x > DIS): # we stell far
                        self.goal_x = blue_marker.pose.position.x
                        self.goal_y = blue_marker.pose.position.y 
                    else: # move from right
#                        self.goal_x = blue_marker.pose.position.x
#                        self.goal_y = blue_marker.pose.position.y +2
                        PAST_FIRST = True

            elif (PAST_FIRST): # we stop see the blue after we saw him
                # trun left until we find blue again
                self.goal_x = 0
                self.goal_y = 1
                time.sleep(3)
                self.goal_x = 4
                self.goal_y = -0.5
                time.sleep(5)
                self.goal_x = 0
                self.goal_y = -1
                time.sleep(3)
                self.goal_x = 4
                self.goal_y = -0.5
                time.sleep(5)
                self.goal_x = 0
                self.goal_y = -1
                time.sleep(4)
                self.goal_x = 5
                self.goal_y = 0
                time.sleep(4)
                PAST_FIRST = False
                END = True
            # elif (timer == ):
            #     FIRST_BLUE = False
            #     self.goal_x = blue_marker.pose.position.x
            #     self.goal_y = (blue_marker.pose.position.y -1) - 2

            elif (green_bouy_flag and red_bouy_flag): # we saw  green & red  
                self.goal_x = (closest_green_marker.pose.position.x + closest_red_marker.pose.position.x)/2
                self.goal_y = ( closest_green_marker.pose.position.y + closest_red_marker.pose.position.y)/2
            elif (green_bouy_flag):
                self.goal_x = closest_green_marker.pose.position.x
                self.goal_y = closest_green_marker.pose.position.y -2
            elif (red_bouy_flag):
                self.goal_x = closest_red_marker.pose.position.x
                self.goal_y = closest_red_marker.pose.position.y +3.5
            elif (END and not green_bouy_flag and not red_bouy_flag):
                #celebrate
                self.goal_yaw = 8
                exit()
            print("red_bouy_flag = " + str(red_bouy_flag) + "green_bouy_flag = " + str(green_bouy_flag)+"yellow_bouy_flag = " + str(yellow_bouy_flag))  
            print ("cv fault ? : goal x" + str(self.goal_x) + "goal y: " + str(self.goal_y))

                

    def pid(self, error, gain):
        # control expect velocity between 1 to -1. we want to go slow so we chose 0.5
        cmd =gain*error
        if (cmd > 0.5):
            cmd= 0.5
        if (cmd< -0.5):
           cmd =-0.5
        return cmd

    def move_towards_the_goal(self):   
        errx =self.goal_x
        erry =self.goal_y
        errz = 0
        erryaw= math.atan(erry/errx)
        #print(errx,erry,errz)
        cmd_vel_x=errx
        cmd_vel_y=erry
        cmd_vel_yaw=0
        #publish the message 
        cmd_vel_msg=Twist()
        cmd_vel_msg.linear.x=cmd_vel_x
        cmd_vel_msg.linear.y=cmd_vel_y
        cmd_vel_msg.linear.z = 0
        cmd_vel_msg.angular.z=cmd_vel_yaw
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



    def stop (self):
        cmd_vel_msg=Twist()
        cmd_vel_msg.linear.x= 0
        cmd_vel_msg.linear.y= 0
        cmd_vel_msg.linear.z = 0
        cmd_vel_msg.angular.z=0
        self.cmd_vel_pub.publish(cmd_vel_msg)


''' 
the state machine is: 
    init - state -1
    run the channel - state 0
    finish - 1
'''

def main():
    rospy.init_node("obsatacle_channel_stupid_node", anonymous=False)
    rate = rospy.Rate(50)
    obsChan = Obsatcle_Chan()
    
    
    while not rospy.is_shutdown() and obsChan.activated:
        if obsChan.state == -1:
            while (not rospy.is_shutdown()) and (len(obsChan.objects_list) < 1): #we know we should see at least pair of bouys
                obsChan.test.publish(obsChan.state)
                rate.sleep()
            obsChan.state = 0 
            

        if obsChan.state == 0: # we have at least 2 bouys 
            obsChan.test.publish(obsChan.state)
            if len(obsChan.objects_list) >= 1:
                obsChan.compute_goal(obsChan.objects_list)
                obsChan.move_towards_the_goal()
            else: # there is no bouys in the object list , so finish the channel
                obsChan.stop()
                time.sleep(3) # TODO CHANGE - wait 3 sec maybe we will bouys in stop pose
                if len(obsChan.objects_list) < 1:
                    obsChan.state = 1

        if obsChan.state == 1:
            obsChan.test.publish(obsChan.state)
            obsChan.stop()
            obsChan.activated = False
            obsChan.status_pub.publish(1)


        rate.sleep()
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass