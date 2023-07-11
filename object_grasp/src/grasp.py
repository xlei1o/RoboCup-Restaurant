#!/usr/bin/env python

import rospy
from tf.transformations import *
from tf2_geometry_msgs import *
import moveit_commander
import sys
from actionlib import SimpleActionClient, SimpleActionServer

from geometry_msgs.msg import Quaternion,Pose
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.msg import Grasp, PickupAction, PickupGoal, PickupResult, MoveItErrorCodes
from moveit_msgs.msg import PlaceAction, PlaceGoal, PlaceResult, PlaceLocation

# asuming we have server to detect the object and return the coordinates
from object_grasp.srv import grasp, graspResponse
from object_grasp.srv import place, placeResponse


# moveit_error_dict = {}
# for name in MoveItErrorCodes.__dict__.keys():
# 	if not name[:1] == '_':
# 		code = MoveItErrorCodes.__dict__[name]
# 		moveit_error_dict[code] = name








class Grasp_Place():
    """
    input: object pose
    """
    def __init__(self):

        self.robot = moveit_commander.RobotCommander()
        self.move_group = moveit_commander.MoveGroupCommander("arm_torso")

    def get_pose(coordinates):
        """
        get the pose of the object

        return: the coordinates of the object >>>> geometry_msgs/Pose
        """
        pose = Pose()
        pose.position.x = coordinates[0]
        pose.position.y = coordinates[1]
        pose.position.z = coordinates[2]

        return pose
        

    def preparation(self,object_pose):
        """
        make tiago put its hands at the right side of the object

        return: operation status(success or fail)

        input: object_pose: the pose of the object >>>> geometry_msgs/Pose
        """
        

        # the final orientation of hand
        object_pose.orientation = Quaternion(0.5,0.5,0.5,0.5)

        # the vertival pose of the hand
        # object_pose.pose.position.z += rospy.get_param('z_axis_offset') # at the position z of the object
        # because the vertival pose is the most safe pose
        object_pose.position.z += 0.1

        # make the grasp pose same as the object pose
        self.grasp_pose = copy.deepcopy(object_pose) 

        # the distance between the hand and the object
        self.grasp_pose.position.y -= 0.1

        # pre-grasp pose
        object_pose.position.y -=0.15 

        rospy.loginfo("pre-grasp pose")

        self.move_group.set_pose_target(object_pose) # quaternion

        # move the manipulator to the pre-grasp pose
        self.move_group.go(wait=True)

        # stop the manipulator at the pre-grasp pose
        self.move_group.stop()

        # delete the pre-grasp pose
        self.move_group.clear_pose_targets()

        return None

    def grasp_cb(self, coordinate):
        """
		:type goal: PickUpPoseGoal
          
		"""
        object_pose = self.get_pose(coordinate)
        error_code = self.grasp(object_pose)
        if error_code != 1:
             grasp_result = 0
        else:
             grasp_result = 1
        
        return graspResponse(grasp_result)
    
    def place_cb(self, object_pose):
        """
		:type goal: PickUpPoseGoal
		"""
        error_code = self.place(object_pose)
        if error_code != 1:
             place_result = 0
        else:
             place_result = 1
        
        return placeResponse(place_result)

    
    def grasp(self,object_pose):
        """
        pick up the object

        object_pose:Pose
        """

        #move to the grasp pose

        self.preparation(object_pose)

        rospy.loginfo("Object pose: %s", object_pose)

        self.move_group.set_pose_target(object_pose)

        self.move_group.go(wait=True)

        self.move_group.stop()

        self.move_group.clear_pose_targets()


        # close the gripper
        rospy.loginfo("close the gripper")

        self.close_gripper()

        rospy.sleep(1)

        # define post-grasp pose
        post_grasp_pose = copy.deepcopy(self.grasp_pose)

        post_grasp_pose.position.z += rospy.get_param('grasp_height') # ToDO: determine the value

        # move to the post-grasp pose
        rospy.loginfo("post-grasp pose")

        self.move_group.set_pose_target(post_grasp_pose)

        self.move_group.go(wait=True)

        self.move_group.stop()

        self.move_group.clear_pose_targets()

        response = EmptyResponse()

        return response


    def place(self,object_pose):

        self.preparation(object_pose)

        rospy.loginfo("place the object")

        self.move_group.set_pose_target(object_pose)

        self.move_group.go(wait=True)

        self.move_group.stop()

        self.move_group.clear_pose_targets()

        # open the gripper
        rospy.loginfo("open the gripper")

        self.open_gripper()

        # move away from the table

        rospy.loginfo("move away from the table")

        object_pose.position.z += 0.2 # ToDO: determine the value

        self.move_group.set_pose_target(object_pose)

        self.move_group.go(wait=True)

        self.move_group.stop()

        self.move_group.clear_pose_targets()

        rospy.sleep(1) 

        response = EmptyResponse()

        return response
       



    def close_gripper():
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

    def open_gripper():

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

    gp=Grasp_Place()

    # grasp_service = rospy.Service("restaurant/grasp_object", grasp , gp.grasp_cb)

    # place_service = rospy.Service("restaurant/place_object", place , gp.place_cb)
    

    
    # pre_object_service = rospy.Service(
    #     "restaurant/pre_object", Pre_object, a.pre_grasp)
    
    # grasp_object_service = rospy.Service(
    #     "restaurant/grasp_object", Empty, a.grasp)
    
    # place_object_service = rospy.Service(
    #     "restaurant/place_object", Empty, a.place)
    
    # ps = Pose()
    # ps.position.x = 0.5
    # ps.position.y = 0.5
    # ps.position.z = 0.5
    # ps.orientation.w = 0.0
    # while not rospy.is_shutdown():
    #     gp.grasp(ps)
    #     rospy.sleep(5.0)

    pp = Pose()
    pp.position.x = 0.0
    pp.position.y = 0.0
    pp.position.z = 0.0
    pp.orientation.w = 0.0
    while not rospy.is_shutdown():
        gp.place(pp)
        rospy.sleep(5.0)


    
    # rospy.loginfo("Grasp_Object is ready.")

    #gp.open_gripper()

    rospy.spin()





