import rospy
import smach
import actionlib
import cv2

from std_msgs.msg import Float64
from control_msgs.msg import PointHeadActionGoal
from geometry_msgs.msg import PointStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from pal_interaction_msgs.msg import TtsAction, TtsGoal
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from darknet_ros_msgs.msg import BoundingBoxes
from objects_msgs.msg import objects, single

from restaurant import functions


class Say(smach.State):
    def __init__(self, text):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.text = text
    def execute(self, userdata):
        if functions.Speak(self.text):
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
        goal.target_pose.header.frame_id = "map" 
        goal.target_pose.pose.position.x = self.coordinate['x']
        goal.target_pose.pose.position.y = self.coordinate['y']
        goal.target_pose.pose.orientation.w = self.coordinate['w']
        client.send_goal(goal)
        client.wait_for_result()
        if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            return 'success'
        else:
            return 'failure'


class Calling(smach.State):
    def __init__(self, hd):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.hd = hd

    def execute(self, userdata):
        rospy.loginfo('Looking for customer raising hand...')
        rospy.sleep(10)  
        if self.hd.result():       
            return 'success'
        else:
            return 'failure'


class LookAround(smach.State):
    def __init__(self, table):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.table = table
        self.pub_head_topic = rospy.Publisher('/head_controller/point_head_action/goal', PointHeadActionGoal, queue_size=1)
    def execute(self, userdata):
        # client = actionlib.SimpleActionClient('/head_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        # client.wait_for_server()
        # goal = FollowJointTrajectoryGoal()
        # goal.trajectory.joint_names = ['head_1_joint', 'head_2_joint']
        # point = JointTrajectoryPoint()
        # point.positions = [self.pan_angle, 0.0]
        # point.time_from_start = rospy.Duration(1.0)
        # goal.trajectory.points.append(point)
        # client.send_goal(goal)
        # client.wait_for_result()
        # if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        #     return 'success'
        # else:
        #     return 'failure'
        phag = PointHeadActionGoal()
        phag.header.frame_id = "/map"
        phag.goal.max_velocity = 1.0
        phag.goal.min_duration = rospy.Duration(0.2)
        phag.goal.target.header.frame_id = "/map"
        phag.goal.pointing_axis.x = 1.0
        phag.goal.pointing_frame = "/head_2_link"

        phag.goal.target.point.x = self.table['x']
        phag.goal.target.point.y = self.table['y']
        # phag.goal.target.point.x = 1.23
        # phag.goal.target.point.y = 1.94
        phag.goal.target.point.z = 0.9
        # rospy.loginfo("Sending: " + str(phag))
        self.pub_head_topic.publish(phag)

        return 'success'
        
class ObjectDetection(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure','preempted'],input_keys=['grasp_ready', 'exist_objects'], output_keys=['exist_objects'])
        self.object_pub = rospy.Publisher('/restaurant/objects', objects, queue_size=10)
        self.names = None
        _ = functions.HeadAction(0.0, -0.8)

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
            _ = functions.HeadAction(0.0, 0.0)
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
            _ = functions.Speak('sorry we do not have '+ self.requre)
            return 'failure'
            
