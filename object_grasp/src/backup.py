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

from spherical_grasps_server import SphericalGrasps

# asuming we have server to detect the object and return the coordinates
from object_grasp.srv import grasp,graspResponse
from object_grasp.srv import place,placeResponse


moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
	if not name[:1] == '_':
		code = MoveItErrorCodes.__dict__[name]
		moveit_error_dict[code] = name



def createPickupGoal(group="arm_torso", target="part",
					 grasp_pose=PoseStamped(),
					 possible_grasps=[]):
	""" Create a PickupGoal with the provided data"""
	pug = PickupGoal()
	pug.target_name = target
	pug.group_name = group
	pug.possible_grasps.extend(possible_grasps)
	pug.allowed_planning_time = 35.0
	pug.planning_options.planning_scene_diff.is_diff = True
	pug.planning_options.planning_scene_diff.robot_state.is_diff = True
	pug.planning_options.plan_only = False
	pug.planning_options.replan = True
	pug.planning_options.replan_attempts = 10  # 10
	pug.allowed_touch_objects = []
	

	return pug

def createPlaceGoal(place_pose,
					place_locations,
					group="arm_torso",
					target="part"):
	"""Create PlaceGoal with the provided data"""
	placeg = PlaceGoal()
	placeg.group_name = group
	placeg.attached_object_name = target
	placeg.place_locations = place_locations
	placeg.allowed_planning_time = 15.0
	placeg.planning_options.planning_scene_diff.is_diff = True
	placeg.planning_options.planning_scene_diff.robot_state.is_diff = True
	placeg.planning_options.plan_only = False
	placeg.planning_options.replan = True
	placeg.planning_options.replan_attempts = 10

	return placeg



class Grasp_Place():
    """
    input: object pose
    """
    def __init__(self):
        self.sg = SphericalGrasps()
        self.pickup_ac = SimpleActionClient('/grasp', PickupAction)
        self.place_ac = SimpleActionClient('/place', PlaceAction)
        self.clear_octomap_srv = rospy.ServiceProxy(
			'/clear_octomap', Empty)
        self.clear_octomap_srv.wait_for_service()
        self.robot = moveit_commander.RobotCommander()
        self.move_group = moveit_commander.MoveGroupCommander("arm_torsor")

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
        

    # def preparation(self,object_pose):
    #     """
    #     make tiago put its hands at the right side of the object

    #     return: operation status(success or fail)

    #     input: object_pose: the pose of the object >>>> geometry_msgs/Pose
    #     """
        

    #     # the final orientation of hand
    #     object_pose.pose.orientation = Quaternion(0.5,0.5,0.5,0.5)

    #     # the vertival pose of the hand
    #     # object_pose.pose.position.z += rospy.get_param('z_axis_offset') # at the position z of the object
    #     # because the vertival pose is the most safe pose
    #     object_pose.pose.position.z += rospy.get_param('z_axis_offset')

    #     # make the grasp pose same as the object pose
    #     self.grasp_pose = copy.deepcopy(object_pose.pose) 

    #     # the distance between the hand and the object
    #     self.grasp_pose.position.y -= rospy.get_param('y_axis_offset')

    #     # pre-grasp pose
    #     object_pose.position.y -=0.15 

    #     rospy.loginfo("pre-grasp pose")

    #     self.move_group.set_pose_target(object_pose.pose) # quaternion

    #     # move the manipulator to the pre-grasp pose
    #     self.move_group.go(wait=True)

    #     # stop the manipulator at the pre-grasp pose
    #     self.move_group.stop()

    #     # delete the pre-grasp pose
    #     self.move_group.clear_pose_targets()

    #     response = ApproachObjectResponse()
    #     response.result = True
    #     return response

    def grasp_cb(self, object_pose):
        """
		:type goal: PickUpPoseGoal
		"""
        error_code = self.grasp_object(object_pose)
        if error_code != 1:
             grasp_result = 0
        else:
             grasp_result = 1
        
        return graspResponse(grasp_result)
    
    def place_cb(self, object_pose):
        """
		:type goal: PickUpPoseGoal
		"""
        error_code = self.place_object(object_pose)
        if error_code != 1:
             place_result = 0
        else:
             place_result = 1
        
        return graspResponse(place_result)

    
    def grasp(self,object_pose):
        """
        pick up the object

        object_pose:Pose
        """

        # move to the grasp pose

        # rospy.loginfo("Object pose: %s", object_pose.pose)

        # self.move_group.set_pose_target(object_pose)

        # self.move_group.go(wait=True)

        # self.move_group.stop()

        # self.move_group.clear_pose_targets()

        # the Second grasp
        possible_grasps = self.sg.create_grasps_from_object_pose(object_pose)
        self.pickup_ac
        goal = createPickupGoal(
			"arm_torso", "part", object_pose, possible_grasps)
        self.pickup_ac.send_goal(goal)
        self.pickup_ac.wait_for_result()
        result = self.pickup_ac.get_result()
        rospy.logdebug("Using torso result: " + str(result))
        return result.error_code.val





        # # close the gripper
        # rospy.loginfo("close the gripper")

        # self.close_gripper()

        # rospy.sleep(1)

        # # define post-grasp pose
        # post_grasp_pose = copy.deepcopy(self.grasp_pose)

        # post_grasp_pose.position.z += rospy.get_param('grasp_height') # ToDO: determine the value

        # # move to the post-grasp pose
        # rospy.loginfo("post-grasp pose")

        # self.move_group.set_pose_target(post_grasp_pose)

        # self.move_group.go(wait=True)

        # self.move_group.stop()

        # self.move_group.clear_pose_targets()

        # response = EmptyResponse()

        # return response


    def place(self,object_pose):

        # rospy.loginfo("place the object")

        # self.move_group.set_pose_target(table_position)

        # self.move_group.go(wait=True)

        # self.move_group.stop()

        # self.move_group.clear_pose_targets()

        # # open the gripper
        # rospy.loginfo("open the gripper")

        # self.open_gripper()

        # # move away from the table

        # rospy.loginfo("move away from the table")

        # table_position.position.z += 0.2 # ToDO: determine the value

        # self.move_group.set_pose_target(table_position)

        # self.move_group.go(wait=True)

        # self.move_group.stop()

        # self.move_group.clear_pose_targets()

        # rospy.sleep(1) 

        # response = EmptyResponse()

        # return response
        self.clear_octomap_srv.call(EmptyRequest())
        possible_placings = self.sg.create_placings_from_object_pose(
			object_pose)
		# Try only with arm
        rospy.loginfo("Trying to place using only arm")
        goal = createPlaceGoal(
			object_pose, possible_placings, "arm", "part")

        rospy.loginfo("Waiting for result")
        self.place_ac.wait_for_result()
        result = self.place_ac.get_result()
        rospy.loginfo(str(moveit_error_dict[result.error_code.val]))

        if str(moveit_error_dict[result.error_code.val]) != "SUCCESS":
            rospy.loginfo(
                "Trying to place with arm and torso")
            # Try with arm and torso
            goal = createPlaceGoal(
                object_pose, possible_placings, "arm_torso", "part", self.links_to_allow_contact)
            rospy.loginfo("Sending goal")
            self.place_ac.send_goal(goal)
            rospy.loginfo("Waiting for result")

            self.place_ac.wait_for_result()
            result = self.place_ac.get_result()
            rospy.logerr(str(moveit_error_dict[result.error_code.val]))
		
		# print result
        rospy.loginfo(
            "Result: " +
            str(moveit_error_dict[result.error_code.val]))
        rospy.loginfo("Removing previous 'part' object")
        self.scene.remove_world_object("part")

        return result.error_code.val




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

    grasp_service = rospy.Service("restaurant/grasp_object", grasp , gp.grasp_cb)

    place_service = rospy.Service("restaurant/place_object", place , gp.place_cb)
    

    
    # pre_object_service = rospy.Service(
    #     "restaurant/pre_object", Pre_object, a.pre_grasp)
    
    # grasp_object_service = rospy.Service(
    #     "restaurant/grasp_object", Empty, a.grasp)
    
    # place_object_service = rospy.Service(
    #     "restaurant/place_object", Empty, a.place)
    
    # ps = Pose()
    # ps.header.frame_id = 'base_footprint'
    # ps.pose.position.x = 1.0
    # ps.pose.position.y = 0.0
    # ps.pose.position.z = 1.0
    # ps.pose.orientation.w = 1.0
    # while not rospy.is_shutdown():
    #     a.preparation(ps)
    #     rospy.sleep(1.0)
    
    # rospy.loginfo("Grasp_Object is ready.")

    # a.open_gripper()

    rospy.spin()





