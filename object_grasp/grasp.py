#!/usr/bin/env python

import rospy
from tf.transformations import *
from tf2_geometry_msgs import *
import moveit_commander
import sys

from geometry_msgs.msg import Quaternion
from std_srvs.srv import Empty, EmptyResponse
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# asuming we have server to detect the object and return the coordinates
from .srv import ObjectDetection, ObjectDetectionResponse


class Grasp:
    def __init__(self):
        self.target_abs_pose = None
        self.robot = moveit_commander.RobotCommander()
        self.grasp_pose = None
        self.move_group = moveit_commander.MoveGroupCommander("arm_torsor")

    def preparation(self,object_pose):
        """
        make tiago put its hands at the right side of the object

        return: operation status(success or fail)
        """
        

        # the final orientation of hand
        object_pose.pose.orientation = Quaternion(0.5,0.5,0.5,0.5)

        # the vertival pose of the hand
        object_pose.pose.position.z += rospy.get_param('z_axis_offset') # at the position z of the object

        self.grasp_pose = copy.deepcopy(object_pose.pose) 

        # the distance between the hand and the object
        self.grasp_pose.position.y -= rospy.get_param('y_axis_offset') 

        # pre-grasp pose
        object_pose.position.y -=0.15 

        rospy.loginfo("pre-grasp pose: {}".format(self.grasp_pose))

        self.move_group.set_pose_target(object_pose.pose) # quaternion

        # move the manipulator to the pre-grasp pose
        self.move_group.go(wait=True)

        # stop the manipulator at the pre-grasp pose
        self.move_group.stop()

        # delete the pre-grasp pose
        self.move_group.clear_pose_targets()

        return True
    
    def grasp(self,msg):
        """
        pick up the object
        """

        # move to the grasp pose
        rospy.loginfo("grasp pose: {}".format(self.grasp_pose))

        self.move_group.set_pose_target(self.grasp_pose)

        self.move_group.go(wait=True)

        self.move_group.stop()

        self.move_group.clear_pose_targets()


        # close the gripper
        rospy.loginfo("close the gripper")

        close_gripper()




    def close_gripper(self):
        """
        close the gripper
        """
        # close the gripper
        gripper_controller = rospy.Publisher('/gripper_controller/command', JointTrajectory, queue_size=1)

        # set a loop to close the gripper
        for i in range(10):
            trajctory = JointTrajectory()
            trajctory.joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']

            point = JointTrajectoryPoint()
            # gripper joints cfg
            point.positions = [0.0, 0.0]

            # set the time of the gripper to close
            point.time_from_start = rospy.Duration(0.5) # ToDO

            trajctory.points.append(point)

            gripper_controller.publish(trajctory)

            rospy.sleep(0.5)




