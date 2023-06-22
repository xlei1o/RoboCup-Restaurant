import rospy
import smach
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from pal_interaction_msgs.msg import TtsAction, TtsGoal
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class Say(smach.State):
    def __init__(self, text):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.text = text
    def execute(self, userdata):
        rospy.loginfo("I'll say: " + self.text)
        client = actionlib.SimpleActionClient('/tts', TtsAction)
        client.wait_for_server()
        goal = TtsGoal()
        goal.rawtext.text = self.text
        goal.rawtext.lang_id = "en_GB"
        client.send_goal_and_wait(goal)
        if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            return 'success'
        else:
            return 'failure'


class Navigation(smach.State):
    def __init__(self, coordinate):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.coordinate = coordinate
    def execute(self, userdata):
        rospy.loginfo('Navigating...')
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "base_footprint" 
        goal.target_pose.pose.position.x = 0.5
        goal.target_pose.pose.position.y = 0.0
        goal.target_pose.pose.orientation.w = 1.0
        client.send_goal(goal)
        client.wait_for_result()
        if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            return 'success'
        else:
            return 'failure'


class Calling(smach.State):
    def __init__(self,hd):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.hd = hd
    def execute(self, userdata):
        rospy.loginfo('Looking for customer raising hand...')
        # hand_detection_node = HandDetection()
        # rospy.sleep(5)
        # hand_detection_node.listen()
        rospy.sleep(5)
        if self.hd.result():
            return 'success'
        else:
            return 'failure'


class LookAround(smach.State):
    def __init__(self, pan_angle):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.pan_angle = pan_angle

    def execute(self, userdata):
        client = actionlib.SimpleActionClient('/head_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        client.wait_for_server()
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ['head_1_joint', 'head_2_joint']
        point = JointTrajectoryPoint()
        point.positions = [self.pan_angle, 0.0]
        point.time_from_start = rospy.Duration(1.0)
        goal.trajectory.points.append(point)
        client.send_goal(goal)
        client.wait_for_result()
        if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            return 'success'
        else:
            return 'failure'
