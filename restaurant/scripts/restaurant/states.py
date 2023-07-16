import rospy
import smach
import actionlib

from control_msgs.msg import PointHeadActionGoal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from darknet_ros_msgs.msg import BoundingBoxes
from objects_msgs.msg import objects, single
from wit_ros.srv import ListenAndInterpret, ListenAndInterpretResponse
from customer_interest_detection.srv import Customer_Interest
from restaurant import functions
from object_grasp.srv import grasp, place


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
    def __init__(self, coordinate=None):
        smach.State.__init__(self, outcomes=['success', 'failure', 'preempted'],
                             input_keys=['grasp_ready'],
                             output_keys=['server_pos'])
        self.coordinate = coordinate

    def execute(self, userdata):
        rospy.loginfo('Navigating...')
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        if self.coordinate is not None:
            goal.target_pose.pose.position.x = self.coordinate['x']
            goal.target_pose.pose.position.y = self.coordinate['y']
            goal.target_pose.pose.orientation.z = self.coordinate['z']
            goal.target_pose.pose.orientation.w = self.coordinate['w']
            client.send_goal(goal)
            client.wait_for_result()
            if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                userdata.server_pos = [
                    self.coordinate['x'], self.coordinate['y'], self.coordinate['z'], self.coordinate['w']]
                return 'success'
            else:
                return 'failure'
        else:
            goal.target_pose.pose.position.x = userdata.server_pos[0]
            goal.target_pose.pose.position.y = userdata.server_pos[1]
            goal.target_pose.pose.orientation.z = userdata.server_pos[2]
            goal.target_pose.pose.orientation.w = userdata.server_pos[3]
            client.send_goal(goal)
            client.wait_for_result()
            if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                return 'preempted'
            else:
                return 'failure'


class Calling(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure'])

    def execute(self, userdata):
        functions.HeadAction(0.0, 0.0)
        rospy.wait_for_service('/restaurant/customer_interest')
        service_proxy = rospy.ServiceProxy(
            '/restaurant/customer_interest', Customer_Interest)
        response = service_proxy()
        if response.customer_interest:
            return 'success'
        else:
            return 'failure'


class LookAround(smach.State):
    def __init__(self, pan):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.pan = pan

    def execute(self, userdata):
        return functions.HeadAction(self.pan, 0.0)


class ObjectDetection(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure', 'preempted'],
                             input_keys=['grasp_ready', 'exist_objects'],
                             output_keys=['exist_objects'])
        # self.object_pub = rospy.Publisher('/restaurant/objects', objects, queue_size=10)
        self.names = None

    def callback(self, box):
        functions.HeadAction(0.0, -0.6)
        rospy.sleep(1.0)
        number = len(box.bounding_boxes)
        self.names = []
        for i in range(number):
            self.names.append(box.bounding_boxes[i].Class)

    def execute(self, userdata):
        rospy.sleep(5)
        rospy.Subscriber('/darknet_ros/bounding_boxes',
                         BoundingBoxes, self.callback)
        rospy.wait_for_message('/darknet_ros/bounding_boxes', BoundingBoxes)
        userdata.exist_objects = self.names
        rospy.loginfo(userdata.exist_objects)
        if userdata.grasp_ready:
            return 'preempted'
        else:
            functions.HeadAction(0.0, 0.0)
            return 'success'


class Conversation(smach.State):
    def __init__(self, text):
        smach.State.__init__(self, outcomes=['success', 'failure'],
                             input_keys=['exist_objects'],
                             output_keys=['require_object', 'grasp_ready'])
        self.text = text

    def execute(self, userdata):
        functions.Speak(self.text)
        rospy.sleep(0.1)
        while not rospy.is_shutdown():
            while not rospy.is_shutdown():
                print(userdata.exist_objects)
                rospy.wait_for_service('/restaurant/wit/listen_interpret')
                service_proxy = rospy.ServiceProxy(
                    '/restaurant/wit/listen_interpret', ListenAndInterpret)
                response = service_proxy()
                if response.result != 'nothing':
                    break
                else:
                    functions.Speak('sorry please say again loudly')
            if response.result in userdata.exist_objects:
                functions.Speak('okay ' + response.result +
                                ' please wait for a moment')
                userdata.require_object = response.result
                userdata.grasp_ready = True
                break
            else:
                functions.Speak('sorry we do not have ' +
                                response.result + ' please order something else')

        rospy.loginfo(response.result)
        return 'success'


class Pickup(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure'],
                             input_keys=['require_object'])
        self.require = None
        self.coordinate = None

    def callback(self, msg):
        for obj in msg.Objects:
            if self.require == obj.name:
                self.coordinate = [obj.x, obj.y, obj.z]

    def execute(self, userdata):
        self.require = userdata.require_object
        rospy.Subscriber('/restaurant/objects', objects, self.callback)
        rospy.wait_for_message('/restaurant/objects', objects)
        request = grasp()
        request.x, request.y, request.z = self.coordinate[0], self.coordinate[1], self.coordinate[2]
        rospy.wait_for_service('/restaurant/grasp_object')
        service_proxy = rospy.ServiceProxy('/restaurant/grasp_object', grasp)
        if service_proxy(request):
            return 'success'
        else:
            return 'failure'


class Place(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure'])

    def execute(self, userdata):
        rospy.wait_for_service('/restaurant/place_object')
        service_proxy = rospy.ServiceProxy('/restaurant/place_object', place)
        if service_proxy():
            return 'success'
        else:
            return 'failure'
