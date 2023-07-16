import rospy
import numpy as np
import actionlib

from std_msgs.msg import Float64
from control_msgs.msg import PointHeadActionGoal
from geometry_msgs.msg import PointStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from pal_interaction_msgs.msg import TtsAction, TtsGoal
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from darknet_ros_msgs.msg import BoundingBoxes
from objects_msgs.msg import objects, single
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from sensor_msgs.msg import JointState


def Speak(text):
    rospy.loginfo("I'll say: " + text)
    client = actionlib.SimpleActionClient('/tts', TtsAction)
    client.wait_for_server()
    goal = TtsGoal()
    goal.rawtext.text = text
    goal.rawtext.lang_id = "en_GB"
    client.send_goal_and_wait(goal)
    if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        return True
    else:
        return False
    
def HeadAction(pan_angle, til_angle = 0.0):
    client = actionlib.SimpleActionClient('/head_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    client.wait_for_server()
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = ['head_1_joint', 'head_2_joint']
    point = JointTrajectoryPoint()
    point.positions = [pan_angle, til_angle]
    point.time_from_start = rospy.Duration(1.0)
    goal.trajectory.points.append(point)
    client.send_goal(goal)
    client.wait_for_result()
    if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        return 'success'
    else:
        return 'failure'
    
def ArmAction(target_angle=None):
    if target_angle is not None:
        group_arm_torso = MoveGroupCommander("arm_torso")
        group_arm_torso.set_planner_id("SBLkConfigDefault")
        group_arm_torso.set_start_state_to_current_state()
        group_arm_torso.set_max_velocity_scaling_factor(1.0)

        joint_names = group_arm_torso.get_active_joints()

        rospy.loginfo("Setting joint targets...")
        for joint in joint_names:
            if joint in target_angle:
                rospy.loginfo("{} goal position: {}".format(joint, target_angle[joint]))
                group_arm_torso.set_joint_value_target(joint, target_angle[joint])
        rospy.loginfo("Planning...")
        group_arm_torso.set_planning_time(5.0)
        plan = group_arm_torso.plan()
        if not plan:
            raise RuntimeError("No plan found")
        rospy.loginfo("Plan found in {} seconds".format(plan.planning_time))
        rospy.loginfo("Executing the plan...")
        group_arm_torso.go(wait=True)
        rospy.loginfo("Motion duration: {} seconds".format(group_arm_torso.get_seconds()))
    else:
        client = actionlib.SimpleActionClient("play_motion", PlayMotionAction)
        client.wait_for_server()
        rospy.wait_for_message("joint_states", JointState)
        rospy.sleep(3.0)
        rospy.loginfo("Tuck arm...")
        goal = PlayMotionGoal()
        goal.motion_name = 'home'
        goal.skip_planning = False
        client.send_goal(goal)
        client.wait_for_result(rospy.Duration(10.0))
        rospy.loginfo("Arm tucked.")