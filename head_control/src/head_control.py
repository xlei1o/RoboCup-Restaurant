#! /usr/bin/env python
import roslib
import rospy
import actionlib
import time

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from control_msgs.msg import PointHeadAction, PointHeadActionGoal, PointHeadGoal
from geometry_msgs.msg import PointStamped, Point
from sensor_msgs.msg import CameraInfo



class Head:
    def __init__(self):
        self.cameraFrame = String()
        self.cameraFrame = "/xtion_rgb_optical_frame"
        self.imageTopic = String()
        self.imageTopic = "/xtion/rgb/image_raw"
        self.cameraInfoTopic = String()
        self.cameraInfoTopic = "/xtion/rgb/camera_info"

    def goal_send(self,client,X,Y,r):
        """
        X: x coordinate of the object
        Y: y coordinate of the object
        r: damping factor
        """
        latestImageStamp = rospy.Time()
        goal = PointHeadGoal()
        pointStamped = PointStamped()

        pointStamped.header.frame_id = self.cameraFrame
        pointStamped.header.stamp = latestImageStamp
        pointStamped.point.x = X 
        pointStamped.point.y = Y 
        pointStamped.point.z = 1.0
        goal.pointing_frame = self.cameraFrame
        goal.pointing_axis.x = 0.0
        goal.pointing_axis.y = 0.0
        goal.pointing_axis.z = 1.0
        goal.min_duration = rospy.Duration(0.5/r)
        goal.max_velocity = 1.5
        goal.target = pointStamped

        client.send_goal(goal)
        client.wait_for_result(rospy.Duration(1.0/r))
        rospy.sleep(0.2/r)
    

    def turning(self,client,num,d):
        """
        Keep the head always facing the front
        ######################################
        x - image width converted from (0)-(640) to (-0.61376)-(0.61376) 
        y - image height converted from (0)-(480) to (-0.456729)-(0.456729)
        The center of the image is in 0,0 
        d: 0-left 1-front 2-right
        """

        rospy.set_param("/head_state",{'p': 0, 'dire': d}) 
        print("current dire: ", d)

        Y = -0.1/6 #the camera slowly goes down

        if num == 1 and d > 0: #body is on the left
            X = -(0.61376/2) #left
            new_d = d-1
            print("next dire: ", new_d)
            wait_r = 2
            print("done operation: ",self.goal_send(client,X,Y,wait_r)) 
        elif num == 2 and d < 2:
            X = 0.61376/2 #right
            new_d = d+1
            print("next dire: ", new_d)
            wait_r = 2
            print("done operation: ",self.goal_send(client,X,Y,wait_r))
        elif num == 3 and d != 1: 
            if d < 1:
                X = 0.61376/2 #right
                new_d = d+1
            else:
                X = -(0.61376/2) #left
                new_d = d-1
            print("next dire: ", new_d)
            wait_r = 2
            print("done operation: ",self.goal_send(client,X,Y,wait_r))
        else:
            new_d = d
            print("next dire: ", new_d)
            print("done operation: null")

        rospy.set_param("/head_state",{'p': 1, 'dire': new_d})
        while d != new_d:
            check = rospy.get_param("/head_state")
            d = check['dire']


    def headturning(self,msg,client):
        """
        p: prermission to move the head
        dire: direction of the head
        z: 1 for left turn, 2 for right turn, 3 for front facing
        """
        check = rospy.get_param("/head_state")
        p = check['p']
        d = check['dire']
        if p == 1:
            if msg.angular.z > 0.0:
                self.turning(client,1,d)
            elif msg.angular.z < 0.0:
                self.turning(client,2,d)
            else:
                self.turning(client,3,d)
        else:
            print("head cannot move.")
    
    def head_control(self,direction):
        """
        input: direction of the head(L,R)
        """
        client = actionlib.SimpleActionClient('head_controller/point_head_action', PointHeadAction)
        client.wait_for_server()

        if direction == "L":
            self.goal_send(client,-(0.61376/2),0.0,2)
        elif direction == "R":
            self.goal_send(client,0.61376/2,0.0,2)
        else:
            print("wrong input")
            



    def controller(self):
        rospy.init_node('head_control_client', anonymous=True)
        client = actionlib.SimpleActionClient('head_controller/point_head_action', PointHeadAction)
        client.wait_for_server()
        
        print("done operation: ", self.goal_send(client,0.0,-(0.456729/3),2)) # make the head a little bit down

        rospy.Subscriber("mobile_base_controller/cmd_vel", Twist, self.headturning, client)

        rospy.spin()

if __name__ == '__main__':
    try:
        p=Head().controller()
    except rospy.ROSInterruptException:
        pass   