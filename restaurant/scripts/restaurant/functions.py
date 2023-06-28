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