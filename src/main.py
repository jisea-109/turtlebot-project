#!/usr/bin/env python
from __future__ import division
import rospy
import numpy as np
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion

import time
import yaml
import os.path
path = os.path.expanduser('~/catkin_ws/src/group_project/world/input_points.yaml')
with open(path,"r") as stream:
    points = yaml.safe_load(stream)
#-------------------------------------------------------------------------------moving bot

import cv2
import sys
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
class GoToPose():
    def __init__(self):

        self.goal_sent = False

	# What to do if shut down (e.g. Ctrl-C or failure)
	rospy.on_shutdown(self.shutdown)

	# Tell the action client that we want to spin a thread by default
	self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
	rospy.loginfo("Wait for the action server to come up")

	self.move_base.wait_for_server()

    def goto(self, pos, quat):

        # Send a goal
        self.goal_sent = True
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = 'map'
	goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

	# Start moving
        self.move_base.send_goal(goal)

	# Allow TurtleBot up to 60 seconds to complete task
	success = self.move_base.wait_for_result(rospy.Duration(60))

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:

            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)

class colourIdentifier():

    def __init__(self):

        # Initialise any flags that signal a colour has been detected (default to false)
        self.green_flag = False
        self.red_flag = False

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.callback)



    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError:
            pass
        # Set the upper and lower bounds of green color
        hsv_green_lower = np.array([30, 40, 40])
        hsv_green_upper = np.array([80, 255, 255])
        # Convert the rgb image into a hsv image
        Hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        mask_for_green = cv2.inRange(Hsv_image, hsv_green_lower, hsv_green_upper)
        # Apply the mask to the original image using the cv2.bitwise_and() method
        green_identifier = cv2.bitwise_and(cv_image,cv_image,mask = mask_for_green)

        contours_green, hierarchy_green = cv2.findContours(mask_for_green,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
        if len(contours_green) > 0:
            c = max(contours_green, key=cv2.contourArea)

            if cv2.contourArea(c) > 2000: # range
                (x, y), radius = cv2.minEnclosingCircle(c)
                center = (int(x),int(y))
                radius = int(radius)
                #cv2.circle(<image>,(<center x>,<center y>),<radius>,<colour (rgb tuple)>,<thickness (defaults to 1)>)
                cv2.circle(Hsv_image,center,radius,hsv_green_upper,1)

                # Then alter the values of any flags
                self.green_flag = True
        else:
            self.green_flag = False

        # Set the upper and lower bounds of red color (rgb value to hsv)
        hsv_red_lower = np.array([0,100,20])
        hsv_red_upper = np.array([0,255,255])
        #filter out except red
        mask_for_red = cv2.inRange(Hsv_image, hsv_red_lower, hsv_red_upper)

        contours_red, hierarchy_red = cv2.findContours(mask_for_red,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
        if len(contours_red) > 0:
            c = max(contours_red, key=cv2.contourArea)
            if cv2.contourArea(c) > 2000:
                (x, y), radius = cv2.minEnclosingCircle(c)
                center = (int(x),int(y))
                radius = int(radius)
                cv2.circle(Hsv_image,center,radius,hsv_red_upper,1)
                self.red_flag = True
        else:
            self.red_flag = False

        cv2.waitKey(3)

class cluedoIdentifier():

    def __init__(self):

        # Initialise any flags that signal a colour has been detected (default to false)
        self.red_flag = False
        self.blue_flag = False
        self.purple_flag = False
        self.yellow_flag = False
        self.scarlet_flag = False
        self.peacock_flag = False
        self.plum_flag = False
        self.mustard_flag = False
        # Remember to initialise a CvBridge() and set up a subscriber to the image topic you wish to use
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.callback)


    def callback(self, data):
        # Convert the received image into a opencv image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError:
            pass

        Hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        hsv_red_lower = np.array([0,100,20])  # scarlet background hsv value
        hsv_red_upper = np.array([10,255,255])

        hsv_blue_lower = np.array([100,50,50]) # peacock background hsv value
        hsv_blue_upper = np.array([140,255,255])

        hsv_purple_lower = np.array([100,35,10]) # palm background hsv value
        hsv_purple_upper = np.array([180,255,255])

        hsv_yellow_lower = np.array([20,100,50]) # mustard background hsv value
        hsv_yellow_upper = np.array([30,255,255])

        hsv_skin_lower = np.array([0,30,50]) # skin hsv value
        hsv_skin_upper = np.array([20,150,255])


        hsv_scarlet_skin_lower = np.array([0,50,48]) # scarlet skin hsv value
        hsv_scarlet_skin_upper = np.array([12,150,255])

        mask_for_red = cv2.inRange(Hsv_image, hsv_red_lower, hsv_red_upper) #filter out except red
        mask_for_blue = cv2.inRange(Hsv_image, hsv_blue_lower, hsv_blue_upper) #filter out except blue
        mask_for_purple = cv2.inRange(Hsv_image, hsv_purple_lower, hsv_purple_upper) #filter out except purple
        mask_for_yellow = cv2.inRange(Hsv_image, hsv_yellow_lower,hsv_yellow_upper) #filter out except yellow
        mask_for_skin = cv2.inRange(Hsv_image, hsv_skin_lower, hsv_skin_upper) #filter out except skin

        mask_for_scarlet_skin = cv2.inRange(Hsv_image, hsv_scarlet_skin_lower, hsv_scarlet_skin_upper) #filter out except scarlet skin

        self.contours_red, hierarchy_red = cv2.findContours(mask_for_red,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE) # find the red contours
        if len(self.contours_red) > 0 and len(self.contours_red) < 30: # if the turtlebot finds the red but it is too big like construction barrels then skip
            c = max(self.contours_red, key=cv2.contourArea)

            if cv2.contourArea(c) > 10: # make sure the range is not too far
                (x, y), radius = cv2.minEnclosingCircle(c)
                center = (int(x),int(y))
                radius = int(radius)
                #cv2.circle(<image>,(<center x>,<center y>),<radius>,<colour (rgb tuple)>,<thickness (defaults to 1)>)
                cv2.circle(Hsv_image,center,radius,hsv_red_upper,1)

                self.red_flag = True # The sign that the turtle bot finds red color

                if self.red_flag == True: # if the turtle bot finds the object with red color
                    # skin_identifier = cv2.bitwise_and(cv_image,cv_image,mask = mask_for_scarlet_skin)
                    self.contours_skin, hierarchy_skin = cv2.findContours(mask_for_scarlet_skin,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)

                    if len(self.contours_skin) > 0 and len(self.contours_skin) < 25: # if the turtle bot finds the object with red color and skin color
                        c = max(self.contours_skin, key=cv2.contourArea)

                        if cv2.contourArea(c) > 10: # the turtle bot thinks it is scarlet
                            (x2, y2), radius_skin = cv2.minEnclosingCircle(c)
                            center_skin = (int(x2),int(y2))
                            radius_skin = int(radius_skin)
                            #cv2.circle(<image>,(<center x>,<center y>),<radius>,<colour (rgb tuple)>,<thickness (defaults to 1)>)
                            cv2.circle(Hsv_image,center_skin,radius_skin,hsv_scarlet_skin_upper,1)

                            self.scarlet_flag = True
                    else:
                        self.scarlet_flag = False
        else: # turn off the flags if there is no object with red color and skin color
            self.red_flag = False
            self.scarlet_flag = False

        self.contours_blue, hierarchy_blue = cv2.findContours(mask_for_blue,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
        if len(self.contours_blue) > 0 and len(self.contours_blue) < 30:
            c = max(self.contours_blue, key=cv2.contourArea)

            if cv2.contourArea(c) > 10:
                (x, y), radius = cv2.minEnclosingCircle(c)
                center = (int(x),int(y))
                radius = int(radius)
                #cv2.circle(<image>,(<center x>,<center y>),<radius>,<colour (rgb tuple)>,<thickness (defaults to 1)>)
                cv2.circle(Hsv_image,center,radius,hsv_blue_upper,1)

                self.blue_flag = True

                if self.blue_flag == True:
                    self.contours_skin, hierarchy_skin = cv2.findContours(mask_for_skin,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
                    if len(self.contours_skin) > 0 and len(self.contours_skin) < 30:
                        c = max(self.contours_skin, key=cv2.contourArea)
                        if cv2.contourArea(c) > 10: # range
                            (x2, y2), radius_skin = cv2.minEnclosingCircle(c)
                            center_skin = (int(x2),int(y2))
                            radius_skin = int(radius_skin)
                            #cv2.circle(<image>,(<center x>,<center y>),<radius>,<colour (rgb tuple)>,<thickness (defaults to 1)>)
                            cv2.circle(Hsv_image,center_skin,radius_skin,hsv_skin_upper,1)

                            self.peacock_flag = True
                    else:
                        self.peacock_flag = False
        else:
            self.blue_flag = False
            self.peacock_flag = False

        # Now the codes are bascially the same but the mask and hsv value are different
        self.contours_purple, hierarchy_purple = cv2.findContours(mask_for_purple,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
        if len(self.contours_purple) > 0 and len(self.contours_purple) < 30:
            c = max(self.contours_purple, key=cv2.contourArea)

            if cv2.contourArea(c) > 10: # range
                (x, y), radius = cv2.minEnclosingCircle(c)
                center = (int(x),int(y))
                radius = int(radius)
                #cv2.circle(<image>,(<center x>,<center y>),<radius>,<colour (rgb tuple)>,<thickness (defaults to 1)>)
                cv2.circle(Hsv_image,center,radius,hsv_purple_upper,1)

                self.purple_flag = True

                if self.purple_flag == True:

                    self.contours_skin, hierarchy_skin = cv2.findContours(mask_for_skin,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
                    if len(self.contours_skin) > 0 and len(self.contours_skin) < 30:
                        c = max(self.contours_skin, key=cv2.contourArea)
                        if cv2.contourArea(c) > 10: # range
                            (x2, y2), radius_skin = cv2.minEnclosingCircle(c)
                            center_skin = (int(x2),int(y2))
                            radius_skin = int(radius_skin)
                            #cv2.circle(<image>,(<center x>,<center y>),<radius>,<colour (rgb tuple)>,<thickness (defaults to 1)>)
                            cv2.circle(Hsv_image,center_skin,radius_skin,hsv_skin_upper,1)

                            self.plum_flag = True
                    else:
                        self.plum_flag = False
        else:
            self.purple_flag = False
            self.plum_flag = False

        self.contours_yellow, hierarchy_yellow = cv2.findContours(mask_for_yellow,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
        if len(self.contours_yellow) > 0 and len(self.contours_yellow) < 30:
            c = max(self.contours_yellow, key=cv2.contourArea)

            if cv2.contourArea(c) > 10: # range
                (x, y), radius = cv2.minEnclosingCircle(c)
                center = (int(x),int(y))
                radius = int(radius)
                #cv2.circle(<image>,(<center x>,<center y>),<radius>,<colour (rgb tuple)>,<thickness (defaults to 1)>)
                cv2.circle(Hsv_image,center,radius,hsv_yellow_upper,1)

                self.yellow_flag = True

                if self.yellow_flag == True:
                    # skin_identifier = cv2.bitwise_and(cv_image,cv_image,mask = mask_for_skin)
                    self.contours_skin, hierarchy_skin = cv2.findContours(mask_for_skin,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
                    if len(self.contours_skin) > 0 and len(self.contours_skin) < 25:
                        c = max(self.contours_skin, key=cv2.contourArea)
                        if cv2.contourArea(c) > 10: # range
                            (x2, y2), radius_skin = cv2.minEnclosingCircle(c)
                            center_skin = (int(x2),int(y2))
                            radius_skin = int(radius_skin)
                            #cv2.circle(<image>,(<center x>,<center y>),<radius>,<colour (rgb tuple)>,<thickness (defaults to 1)>)
                            cv2.circle(Hsv_image,center_skin,radius_skin,hsv_skin_upper,1)

                            self.mustard_flag = True
                    else:
                        self.mustard_flag = False
        else:
            self.yellow_flag = False
            self.mustard_flag = False
        # bitwiseOr = cv2.bitwise_or(mask_for_yellow,mask_for_skin)
        # bitwiseAnd = cv2.bitwise_and(cv_image,cv_image,mask = bitwiseOr)
        # cv2.imshow('red', bitwiseAnd)
        cv2.waitKey(3)

class image_converter(): # save a screenshot and text file that contains the cluedo's name

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("camera/rgb/image_raw",Image,self.callback)

    def callback(self,data):
        try:
            cv2_img = self.bridge.imgmsg_to_cv2(data,"bgr8")
        except CvBridgeError, e:
            print(e)
        else:
            cv2.imwrite("/home/csunix/sc19s2c/catkin_ws/src/group_project/output/cluedo_character.png", cv2_img)
if __name__ == '__main__':
    try:
        rospy.init_node('nav_test', anonymous=True) #navigation
        navigator = GoToPose()
        cI = colourIdentifier()

        x = points['room2_entrance_xy'][0]
        y = points['room2_entrance_xy'][1]
        theta = 0
        position = {'x': x, 'y' : y}
        quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : np.sin(theta/2.0), 'r4' : np.cos(theta/2.0)}

        rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
        navigator.goto(position, quaternion)

        rospy.sleep(1)

        if cI.green_flag == False: # look around to find the green circle
            if cI.red_flag == True:
                pass
            else:
                for i in range(1,10):
                    theta = 0.5 * i
                    quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : np.sin(theta/2.0), 'r4' : np.cos(theta/2.0)}
                    navigator.goto(position, quaternion)
                    if cI.green_flag == True:
                        break
                    if cI.red_flag == True:
                        break

        if cI.green_flag == True: # if the turtlebot find the green circle at the entrance of room 2
            x = points['room2_centre_xy'][0]# SPECIFY X COORDINATE HERE
            y = points['room2_centre_xy'][1]# SPECIFY Y COORDINATE HERE
            theta = 0# SPECIFY THETA (ROTATION) HERE
            position = {'x': x, 'y' : y}
            quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : np.sin(theta/2.0), 'r4' : np.cos(theta/2.0)}
            rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
            navigator.goto(position, quaternion)

        elif cI.red_flag ==True: # if the turtlebot find the green circle instead, go to room 1
            x = points['room1_centre_xy'][0]# SPECIFY X COORDINATE HERE
            y = points['room1_centre_xy'][1]# SPECIFY Y COORDINATE HERE
            theta = 0# SPECIFY THETA (ROTATION) HERE
            position = {'x': x, 'y' : y}
            quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : np.sin(theta/2.0), 'r4' : np.cos(theta/2.0)}

            rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
            navigator.goto(position, quaternion)

        else: # go to room 1 if the turtlebot does not find the green circle nearby
            x = points['room1_entrance_xy'][0]
            y = points['room1_entrance_xy'][1]
            theta = 0
            position = {'x': x, 'y' : y}
            quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : np.sin(theta/2.0), 'r4' : np.cos(theta/2.0)}

            rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
            navigator.goto(position, quaternion)

            if cI.green_flag == False: # look around at the in front of room 1
                if cI.red_flag == True:
                    pass
                else:
                    for i in range(1,10):
                        theta = 0.5 * i
                        quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : np.sin(theta/2.0), 'r4' : np.cos(theta/2.0)}
                        navigator.goto(position, quaternion)
                        if cI.green_flag == True:
                            break
                        if cI.red_flag == True:
                            break

            if cI.green_flag == True: # if the turtlebot find the green circle, then enther the room
                x = points['room1_centre_xy'][0]
                y = points['room1_centre_xy'][1]
                theta = 0
                position = {'x': x, 'y' : y}
                quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : np.sin(theta/2.0), 'r4' : np.cos(theta/2.0)}

                rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
                navigator.goto(position, quaternion)

            elif cI.red_flag == True:# if the turtlebot find the green circle instead, go to room 2
                x = points['room2_centre_xy'][0]
                y = points['room2_centre_xy'][1]
                theta = 0
                position = {'x': x, 'y' : y}
                quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : np.sin(theta/2.0), 'r4' : np.cos(theta/2.0)}

                rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
                navigator.goto(position, quaternion)
        cluedo = cluedoIdentifier()
        stop_flag = 0
        for i in range(1,15):
            theta = 0.5 * i
            quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : np.sin(theta/2.0), 'r4' : np.cos(theta/2.0)}
            navigator.goto(position, quaternion)
            if cluedo.scarlet_flag == True: # if the turtle finds scarlet
                im = image_converter()
                f = open("~/catkin_ws/src/group_project/output/cluedo_character.txt","w")
                f.write("Scarlet")
                f.close()
                stop_flag = 1 # to stop the action after finding cluedo (line 460)
                break
            if cluedo.peacock_flag == True: # if the turtle finds peacock
                im = image_converter()
                f = open("~/catkin_ws/src/group_project/output/cluedo_character.txt","w")
                f.write("Peacock")
                f.close()
                stop_flag = 1
                break
            if cluedo.plum_flag == True: # if the turtle finds plum
                im = image_converter()
                f = open("~/catkin_ws/src/group_project/output/cluedo_character.txt","w")
                f.write("Plum")
                f.close()
                stop_flag = 1
                break
            if cluedo.mustard_flag == True: # if the turtle finds mustard
                im = image_converter()
                f = open("~/catkin_ws/src/group_project/output/cluedo_character.txt","w")
                f.write("Mustard")
                f.close()
                stop_flag = 1
                break

        for i in range(6,28):
            if stop_flag == 1: # stop looking around as the cluedo is already found
                break
            if (i < 11 and i >= 6):
                theta = 0.4 * i - 0.3
                quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : np.sin(theta/2.0), 'r4' : np.cos(theta/2.0)}
                temp_x = x-1
                position = {'x': temp_x, 'y' : y}
                navigator.goto(position, quaternion)
            if (i < 16 and i >= 11):
                theta = 0.4 * i - 0.6
                quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : np.sin(theta/2.0), 'r4' : np.cos(theta/2.0)}
                temp_y = y - 1
                position = {'x': x, 'y' : temp_y}
                navigator.goto(position, quaternion)
            if (i < 21 and i >= 16):
                theta = 0.4 * i - 1.3
                quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : np.sin(theta/2.0), 'r4' : np.cos(theta/2.0)}
                temp_x = x + 1
                position = {'x': temp_x, 'y' : y}
                navigator.goto(position, quaternion)
            if (i <= 28 and i >= 21):
                theta = 0.4 * i - 2.0
                quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : np.sin(theta/2.0), 'r4' : np.cos(theta/2.0)}
                temp_y = y + 2
                temp_x = x + 1
                position = {'x': temp_x, 'y' : temp_y}
                navigator.goto(position, quaternion)
            if cluedo.scarlet_flag == True:
                im = image_converter()
                f = open("~/catkin_ws/src/group_project/output/cluedo_character.txt","w")
                f.write("Scarlet")
                f.close()
                break
            if cluedo.peacock_flag == True: # save a screenshot
                im = image_converter()
                f = open("~/catkin_ws/src/group_project/output/cluedo_character.txt","w")
                f.write("Peacock")
                f.close()
                stop_flag = 1
                break
            if cluedo.plum_flag == True:
                im = image_converter()
                f = open("~/catkin_ws/src/group_project/output/cluedo_character.txt","w")
                f.write("Plum")
                f.close()
                stop_flag = 1
                break
            if cluedo.mustard_flag == True:
                im = image_converter()
                f = open("~/catkin_ws/src/group_project/output/cluedo_character.txt","w")
                f.write("Mustard")
                f.close()
                stop_flag = 1
                break
    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")
