#!/usr/bin/env python

import rospy
from tf.transformations import *
from tf2_geometry_msgs import *
import moveit_commander
import sys
import functools

from geometry_msgs.msg import Quaternion,Pose
from std_srvs.srv import Empty, EmptyResponse
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# asuming we have server to detect the object and return the coordinates
from ..srv import Pre_object,Rel2Abs


class Grasp:
    """
    input: object pose
    """
    def __init__(self):
        self.target_abs_pose = None
        self.robot = moveit_commander.RobotCommander()
        self.grasp_pose = None
        self.move_group = moveit_commander.MoveGroupCommander("arm_torsor")
        
        self.preparation(self)
        self.grasp(self)
        self.place(self)
        self.open_gripper(self)
        self.close_gripper(self)

    def preparation(self,object_pose):
        """
        make tiago put its hands at the right side of the object

        return: operation status(success or fail)

        input: object_pose: the pose of the object >>>> geometry_msgs/Pose
        """
        

        # the final orientation of hand
        object_pose.pose.orientation = Quaternion(0.5,0.5,0.5,0.5)

        # the vertival pose of the hand
        # object_pose.pose.position.z += rospy.get_param('z_axis_offset') # at the position z of the object
        object_pose.pose.position.z +=1

        self.grasp_pose = copy.deepcopy(object_pose.pose) 

        # the distance between the hand and the object
        # self.grasp_pose.position.y -= rospy.get_param('y_axis_offset') 
        self.grasp_pose.position.y -=1

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

        self.close_gripper()

        rospy.sleep(1)

        # define post-grasp pose
        post_grasp_pose = copy.deepcopy(self.grasp_pose)

        post_grasp_pose.position.z += 0.2 # ToDO: determine the value

        # move to the post-grasp pose
        rospy.loginfo("post-grasp pose: {}".format(post_grasp_pose))

        self.move_group.set_pose_target(post_grasp_pose)

        self.move_group.go(wait=True)

        self.move_group.stop()

        self.move_group.clear_pose_targets()


    def place(self,table_position):

        rospy.loginfo("place the object")

        self.move_group.set_pose_target(table_position)

        self.move_group.go(wait=True)

        self.move_group.stop()

        self.move_group.clear_pose_targets()

        # open the gripper
        rospy.loginfo("open the gripper")

        self.open_gripper()

        # move away from the table

        rospy.loginfo("move away from the table")

        table_position.position.y -= 0.2 # ToDO: determine the value

        self.move_group.set_pose_target(table_position)

        self.move_group.go(wait=True)

        self.move_group.stop()

        self.move_group.clear_pose_targets()

        rospy.sleep(1)



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

    def open_gripper(self):

        gripper_controller = rospy.Publisher('/gripper_controller/command', JointTrajectory, queue_size=1)

        for i in range(10):
            trajctory = JointTrajectory()
            trajctory.joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']

            point = JointTrajectoryPoint()
            # gripper joints cfg
            point.positions = [0.044, 0.044]

            # set the time of the gripper to close
            point.time_from_start = rospy.Duration(0.5)

            trajctory.points.append(point)

            gripper_controller.publish(trajctory)

            rospy.sleep(0.2)

if __name__ == "__main__":

    rospy.init_node("grasp_object")

    moveit_commander.roscpp_initialize(sys.argv)

    a = Grasp()

    object_pose = Pose()
    object_pose.position.x =1.0
    object_pose.position.y =1.0
    object_pose.position.z =1.0
    object_pose.orientation.x = 0
    object_pose.orientation.y = 0
    object_pose.orientation.z = 0
    object_pose.orientation.w = 0


    
    pre_object_service = rospy.Service(
        "grasp_object/pre_object", Pre_object, a.preparation(object_pose))
    
    grasp_object_service = rospy.Service(
        "grasp_object/grasp_object", Empty, a.grasp)
    
    # place_object_service = rospy.Service(
    #     "grasp_object/place_object", Empty, Grasp.place)
    
    rospy.loginfo("Grasp_Object is ready.")

    Grasp.open_gripper()

    rospy.spin()





