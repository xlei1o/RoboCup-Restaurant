import rospy
import smach
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from pal_interaction_msgs.msg import TtsAction, TtsGoal
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from darknet_ros_msgs.msg import BoundingBoxes
from objects_msgs.msg import objects, single


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
        
class ObjectDetection(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure','preempted'],input_keys=['grasp_ready', 'exist_objects'], output_keys=['exist_objects'])
        self.object_pub = rospy.Publisher('/restaurant/objects', objects, queue_size=10)
        self.names = None

    def callback(self, box):
        number = len(box.bounding_boxes)
        msg = objects()
        self.names = []
        for i in range(number):
            object = single()
            width = box.bounding_boxes[i].xmax - box.bounding_boxes[i].xmin
            height = box.bounding_boxes[i].xmin - box.bounding_boxes[i].ymin
            object.name = box.bounding_boxes[i].Class
            object.x = box.bounding_boxes[i].xmin + width
            object.y = box.bounding_boxes[i].ymin + height
            msg.Objects.append(object)
            self.names.append(object.name)
        self.object_pub.publish(msg)
        
    def execute(self, userdata):
        rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.callback)
        rospy.wait_for_message('/darknet_ros/bounding_boxes', BoundingBoxes)
        userdata.exist_objects = self.names
        if userdata.grasp_ready:
            return 'preempted'
        else:
            return 'success'


class CheckObjExist(smach.State):
    def __init__(self, requre):
        smach.State.__init__(self, outcomes=['success', 'failure'],
                             input_keys=['exist_objects'],
                             output_keys=['grasp_ready'])
        self.requre = requre

    def execute(self, userdata):
        if self.requre in userdata.exist_objects:
            userdata.grasp_ready = True
            return 'success'
        else: 
            client = actionlib.SimpleActionClient('/tts', TtsAction)
            client.wait_for_server()
            goal = TtsGoal()
            goal.rawtext.text = 'sorry we do not have ' + self.requre
            goal.rawtext.lang_id = "en_GB"
            client.send_goal_and_wait(goal)
            return 'failure'
