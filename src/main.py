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
        self.flag = False

        # Initialise the value you wish to use for sensitivity in the colour detection (10 should be enough)
        self.sensitivity = 10

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
        hsv_green_lower = np.array([40-self.sensitivity, 40, 40])
        hsv_green_upper = np.array([70+self.sensitivity, 255, 255])
        # Convert the rgb image into a hsv image
        Hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        # Filter out everything but a particular colour using the cv2.inRange() method
        mask = cv2.inRange(Hsv_image, hsv_green_lower, hsv_green_upper)
        # Apply the mask to the original image using the cv2.bitwise_and() method
        bitwiseAnd = cv2.bitwise_and(cv_image,cv_image,mask = mask)

        contours, hierarchy = cv2.findContours(mask,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
            if cv2.contourArea(c) > 5000:
                (x, y), radius = cv2.minEnclosingCircle(c)
                center = (int(x),int(y))
                radius = int(radius)
                #cv2.circle(<image>,(<center x>,<center y>),<radius>,<colour (rgb tuple)>,<thickness (defaults to 1)>)
                cv2.circle(Hsv_image,center,radius,hsv_green_upper,1)

                # Then alter the values of any flags
                self.flag = True
        else:
            self.flag = False
        cv2.imshow('Camera', bitwiseAnd)
        cv2.waitKey(3)

if __name__ == '__main__':
    try:
        rospy.init_node('nav_test', anonymous=True) #navigation
        navigator = GoToPose()
        cI = colourIdentifier()

        x = points['room2_entrance_xy'][0] # set the coordinate of room 2
        y = points['room2_entrance_xy'][1]
        theta = 0
        position = {'x': x, 'y' : y}
        quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : np.sin(theta/2.0), 'r4' : np.cos(theta/2.0)}

        rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
        navigator.goto(position, quaternion)

        rospy.sleep(1)
        if cI.flag == False: # look around to find the green circle
            for i in range(1,6):
                theta = 1 * i
                quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : np.sin(theta/2.0), 'r4' : np.cos(theta/2.0)}
                navigator.goto(position, quaternion)
                if cI.flag == True:
                    break
        if cI.flag == True: # if the turtlebot find the green circle at the entrance of room 2
            x = points['room2_centre_xy'][0]# SPECIFY X COORDINATE HERE
            y = points['room2_centre_xy'][1]# SPECIFY Y COORDINATE HERE
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

            if cI.flag == False: # look around at the in front of room 1
                for i in range(1,6):
                    theta = 1 * i
                    quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : np.sin(theta/2.0), 'r4' : np.cos(theta/2.0)}
                    navigator.goto(position, quaternion)
                    if cI.flag == True:
                        break
            if cI.flag == True: # if the turtlebot find the green circle, then enther the room
                x = points['room1_centre_xy'][0]
                y = points['room1_centre_xy'][1]
                theta = 0
                position = {'x': x, 'y' : y}
                quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : np.sin(theta/2.0), 'r4' : np.cos(theta/2.0)}

                rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
                navigator.goto(position, quaternion)

    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")
