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
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import Image
from tf.transformations import quaternion_from_euler
# asuming we have server to detect the object and return the coordinates
from object_grasp.srv import grasp, graspResponse
from object_grasp.srv import place, placeResponse

# get grasp_pose
from grasp_detection.srv import GraspDetection,GraspDetectionRequest, GraspDetectionResponse


# moveit_error_dict = {}
# for name in MoveItErrorCodes.__dict__.keys():
# 	if not name[:1] == '_':
# 		code = MoveItErrorCodes.__dict__[name]
# 		moveit_error_dict[code] = name


def create_collision_object(id,dimensions,pose):
        object = CollisionObject()
        object.id = id
        object.header.frame_id = 'base_footprint' #TODO: be sure about this

        solid = SolidPrimitive()
        solid.type = solid.BOX
        solid.dimensions = dimensions
        object.primitives = [solid]

        object_pose = Pose()
        object_pose.position.x = pose[0]
        object_pose.position.y = pose[1]
        object_pose.position.z = pose[2]

        object.primitive_poses = [object_pose]
        object.operation = object.ADD
        return object






class Grasp_Place():
    """
    input: object pose
    """
    def __init__(self):

        self.robot = moveit_commander.RobotCommander()
        self.move_group = moveit_commander.MoveGroupCommander("arm_torso")
        self.move_group.set_planning_time(15)
        self.scene = moveit_commander.PlanningSceneInterface()

    def get_pose(self,coordinates):
        """
        get the pose of the object

        return: the coordinates of the object >>>> geometry_msgs/Pose
        """
        pose = Pose()
        pose.position.x = coordinates[0]
        pose.position.y = coordinates[1]
        pose.position.z = coordinates[2]

        return pose
    
    def add_collision_objects(self,id,dimentions,pose):
        rospy.loginfo('add_table')
        co = create_collision_object(id,dimentions,pose)
        self.scene.add_object(co)
        return None     

    def preparation(self):
        """
        make tiago put its hands at the right side of the object

        return: operation status(success or fail)

        input: object_pose: the pose of the object >>>> geometry_msgs/Pose
        """
        

        # the final orientation of hand
        # object_pose.orientation = Quaternion(0.56,-0.44,-0.49,0.49)

        # the vertival pose of the hand
        # object_pose.pose.position.z += rospy.get_param('z_axis_offset') # at the position z of the object
        # because the vertival pose is the most safe pose
        # object_pose.position.z += 0.1

        # # make the grasp pose same as the object pose
        # self.grasp_pose = copy.deepcopy(object_pose) 

        # # the distance between the hand and the object
        # self.grasp_pose.position.y -= 0.05

        # # pre-grasp pose
        # object_pose.position.y -=0.1
        # self.grasp_pose = copy.deepcopy(object_pose)
        # self.grasp_pose.position.x = 0.658
        # self.grasp_pose.position.y = 0.058
        # self.grasp_pose.position.z = 0.922

        # rospy.loginfo("pre-grasp pose")

        # self.move_group.set_pose_target(object_pose) # quaternion

        # # move the manipulator to the pre-grasp pose
        # self.move_group.go(wait=True)

        # # stop the manipulator at the pre-grasp pose
        # self.move_group.stop()

        # # delete the pre-grasp pose
        # self.move_group.clear_pose_targets()


        rospy.loginfo("start pre_pose")
        arm_group = moveit_commander.MoveGroupCommander('arm_torso')
        # target_angles = [0.215, 1.57, -1.5, -3.1, 2.29, 1.75, 0.77, -0.19]
        # target_angles = [0.215, 1.61, -0.61, -3.16, 1.92, 1.66, 1.27, 0]
        target_angles = [0.234, 1.57, -1.4, -3.14, 2.16, 1.71, 0.79, -0.12]
        arm_group.set_joint_value_target(target_angles)
        # plan=arm_group.plan()
        arm_group.go()
        arm_group.stop()
        arm_group.clear_pose_targets()
        rospy.sleep(2)

        return None
    

    def postgrasp(self):

        rospy.loginfo("start post_pose")
        arm_group = moveit_commander.MoveGroupCommander('arm_torso')
        target_angles = [0.234, 1.36, 0, -2.81, 1.54, 1.87, 1.34, 0.35]
        arm_group.set_joint_value_target(target_angles)
        arm_group.set_max_acceleration_scaling_factor(0.8)
        arm_group.go()
        arm_group.stop()
        arm_group.clear_pose_targets()
        rospy.sleep(2)

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

        #add the table_storage

        # self.add_collision_objects('table_grasp',[0.8,1.0,0.72],[1.2,0,0.36]) 

        # object_pose.orientation = Quaternion(-0.47,-0.53,-0.47,0.51)
        # object_pose.orientation = Quaternion(0.3095,0.6370,0.6345,0.3090)
        orientation = quaternion_from_euler(3.14/2, 0.0, 0.0) #TODO: how about 0
        object_pose.orientation.x = orientation[0]
        object_pose.orientation.y = orientation[1]
        object_pose.orientation.z = orientation[2]
        object_pose.orientation.w = orientation[3]
        #move to the pre-grasp pose

        # self.preparation()
        self.postgrasp()

        rospy.sleep(3)

        # move to the grasp-pose

        rospy.loginfo("Object pose: %s", object_pose)

        # the sensor of gripper is 5 cm behind
        object_pose.position.z +=0.05

        # setted_workspace = [0.2,-0.28,0.75,0.91,2,1.3]

        # self.move_group.set_workspace(setted_workspace)
        # add grasp waypoints
        scale = 4
        waypoints = []
        wpose = object_pose
        wpose.position.z += scale *0.2
        wpose.position.x -= scale *0.1
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.z -=scale *0.1
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.z -=scale *0.1
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x +=scale *0.1
        waypoints.append(copy.deepcopy(wpose))
        (plan,fraction)=self.move_group.compute_cartesian_path(waypoints,0.01,0.0)
        self.move_group.excute(plan,wait=True)

        #allow replanning
        # self.move_group.allow_replanning(value = True)

        # self.move_group.set_pose_target(object_pose)
    
        # self.move_group.go(wait=True)

        # self.move_group.stop()

        # self.move_group.clear_pose_targets()

        # rospy.sleep(3)


        # close the gripper
        rospy.loginfo("close the gripper")

        self.close_gripper()

        rospy.sleep(1)

        # # define post-grasp pose
        # post_grasp_pose = copy.deepcopy(self.grasp_pose)

        # post_grasp_pose.position.z += 0.3 # ToDO: determine the value

        # # move to the post-grasp pose
        # rospy.loginfo("post-grasp pose")

        # self.move_group.set_pose_target(post_grasp_pose)

        # self.move_group.go(wait=True)

        # self.move_group.stop()

        # self.move_group.clear_pose_targets()

        self.postgrasp()

        # remove the table
        self.scene.remove_world_object("table_grasp")

        response = EmptyResponse()

        return response


    def place(self,object_pose):

        #add the table_place

        self.add_collision_objects('table_place',[0.3,0.6,0.2],[0,0,-0.1]) #TODO: change the paremeter

        self.preparation(object_pose)

        setted_workspace = [0.2,-0.28,0.45,0.91,2,1.2]

        self.move_group.set_workspace(setted_workspace)

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

        object_pose.position.z += 0.4 # ToDO: determine the value

        self.move_group.set_pose_target(object_pose)

        self.move_group.go(wait=True)

        self.move_group.stop()

        self.move_group.clear_pose_targets()

        rospy.sleep(1)

        # remove the table
        self.scene.remove_world_object("table_place")

        response = EmptyResponse()

        return response
       



    def close_gripper(self):
        """
        close the gripper
        """
        # close the gripper
        rospy.loginfo('close gripper')
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
        
        rospy.loginfo('open gripper')
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
    

    
    
    # ps = Pose()
    # ps.position.x = 0.5
    # ps.position.y = 0.5
    # ps.position.z = 0.5
    # ps.orientation.w = 0.0
    # while not rospy.is_shutdown():
    #     gp.grasp(ps)
    #     rospy.sleep(5.0)

    pp = Pose()
    # pp.position.x = 0.7
    # pp.position.y = 0.5
    # pp.position.z = 0.8

    pp.position.x = 0.76
    pp.position.y = -0.315
    pp.position.z = 0.81
    pp.orientation.w = 0.0
    # gp.add_collision_objects('table_place',[0.8,1.0,0.72],[1.1,0,0.36])
    gp.grasp(pp)
    # gp.preparation()
    # gp.postgrasp()
    
    # rospy.loginfo("Grasp_Object is ready.")

    # gp.open_gripper()

    rospy.spin()





