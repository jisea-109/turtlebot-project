#!/usr/bin/env python
from __future__ import division
import rospy
import numpy as np
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion

import yaml
path = '/home/csunix/sc19s2c/catkin_ws/src/group_project/mock_evaluation/worlds/world2/input_points.yaml'
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
        # Remember to initialise a CvBridge() and set up a subscriber to the image topic you wish to use
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.callback)
        # We covered which topic to subscribe to should you wish to receive image data


    def callback(self, data):
        # Convert the received image into a opencv image
        # But remember that you should always wrap a call to this conversion method in an exception handler
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError:
            pass
        # Set the upper and lower bounds of green color
        hsv_green_lower = np.array([30, 40, 40])
        hsv_green_upper = np.array([80, 255, 255])
        # Convert the rgb image into a hsv image
        Hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        # Filter out everything but a particular colour using the cv2.inRange() method
        mask_for_green = cv2.inRange(Hsv_image, hsv_green_lower, hsv_green_upper)
        # Apply the mask to the original image using the cv2.bitwise_and() method
        green_identifier = cv2.bitwise_and(cv_image,cv_image,mask = mask_for_green)

        contours_green, hierarchy_green = cv2.findContours(mask_for_green,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
        if len(contours_green) > 0:
            c = max(contours_green, key=cv2.contourArea)
            # M = cv2.moments(c)
            # try:
            #     cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
            # except ZeroDivisionError:
            #     pass

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

        hsv_red_lower = np.array([0,100,20])
        hsv_red_upper = np.array([0,255,255])
        mask_for_red = cv2.inRange(Hsv_image, hsv_red_lower, hsv_red_upper)
        red_identifier = cv2.bitwise_and(cv_image,cv_image,mask = mask_for_red)
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
        # cv2.imshow('green', green_identifier)
        # cv2.imshow('red', red_identifier)
        cv2.waitKey(3)

class cluedoIdentifier():

    def __init__(self):

        # Initialise any flags that signal a colour has been detected (default to false)
        self.red_flag = False
        self.scarlet_flag = True
        # Remember to initialise a CvBridge() and set up a subscriber to the image topic you wish to use
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.callback)
        # We covered which topic to subscribe to should you wish to receive image data


    def callback(self, data):
        # Convert the received image into a opencv image
        # But remember that you should always wrap a call to this conversion method in an exception handler
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError:
            pass
        hsv_red_lower = np.array([0,100,20])
        hsv_red_upper = np.array([10,255,255])
        hsv_skin_lower = np.array([0,50,48])
        hsv_skin_upper = np.array([12,150,255])
        Hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask_for_red = cv2.inRange(Hsv_image, hsv_red_lower, hsv_red_upper)
        mask_for_skin = cv2.inRange(Hsv_image, hsv_skin_lower, hsv_skin_upper)
        # Apply the mask to the original image using the cv2.bitwise_and() method
        red_identifier = cv2.bitwise_and(cv_image,cv_image,mask = mask_for_red)

        self.contours_red, hierarchy_red = cv2.findContours(mask_for_red,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
        if len(self.contours_red) > 0 and len(self.contours_red) < 30:
            c = max(self.contours_red, key=cv2.contourArea)

            if cv2.contourArea(c) > 10: # range
                (x, y), radius = cv2.minEnclosingCircle(c)
                center = (int(x),int(y))
                radius = int(radius)
                #cv2.circle(<image>,(<center x>,<center y>),<radius>,<colour (rgb tuple)>,<thickness (defaults to 1)>)
                cv2.circle(Hsv_image,center,radius,hsv_red_upper,1)

                self.red_flag = True
                if self.red_flag == True:
                    skin_identifier = cv2.bitwise_and(cv_image,cv_image,mask = mask_for_skin)
                    self.contours_skin, hierarchy_skin = cv2.findContours(mask_for_skin,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
                    if len(self.contours_skin) > 0 and len(self.contours_skin) < 25:
                        c = max(self.contours_skin, key=cv2.contourArea)
                        if cv2.contourArea(c) > 10: # range
                            (x2, y2), radius_skin = cv2.minEnclosingCircle(c)
                            center_skin = (int(x2),int(y2))
                            radius_skin = int(radius_skin)
                            #cv2.circle(<image>,(<center x>,<center y>),<radius>,<colour (rgb tuple)>,<thickness (defaults to 1)>)
                            cv2.circle(Hsv_image,center_skin,radius_skin,hsv_skin_upper,1)

                            self.scarlet_flag = True
                        else:
                            self.scarlet_flag = False
                else:
                    self.red_flag = False
        bitwiseOr = cv2.bitwise_or(mask_for_red,mask_for_skin)
        bitwiseAnd = cv2.bitwise_and(cv_image,cv_image,mask = bitwiseOr)
        cv2.imshow('red', bitwiseAnd)
        cv2.waitKey(3)

class image_converter():

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

        x = points['room2_entrance_xy'][0] # 22222
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
            x = points['room2_centre_xy'][0]# SPECIFY X COORDINATE HERE 222222
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
            if cluedo.scarlet_flag == True:
                im = image_converter()
                f = open("/home/csunix/sc19s2c/catkin_ws/src/group_project/output/cluedo_character.txt","w")
                f.write("Scarlet")
                f.close()
                stop_flag = 1
                break
        for i in range(6,28):
            if stop_flag == 1:
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
                f = open("/home/csunix/sc19s2c/catkin_ws/src/group_project/output/cluedo_character.txt","w")
                f.write("Scarlet")
                f.close()
                break

    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")
