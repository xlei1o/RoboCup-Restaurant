import rospy
import smach
import actionlib

from control_msgs.msg import PointHeadActionGoal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from darknet_ros_msgs.msg import BoundingBoxes
from objects_msgs.msg import objects, single
from wit_ros.srv import ListenAndInterpret, ListenAndInterpretResponse
from object_detect.srv import Customer_Interest
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
        print(self.coordinate['y'])
        goal.target_pose.pose.position.y = self.coordinate['y']
        goal.target_pose.pose.orientation.z = self.coordinate['z']
        goal.target_pose.pose.orientation.w = self.coordinate['w']
        client.send_goal(goal)
        client.wait_for_result()
        if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            return 'success'
        else:
            return 'failure'


class Calling(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure'])

    def execute(self, userdata):

        rospy.wait_for_service('/restaurant/customer_interest')
        service_proxy = rospy.ServiceProxy('/restaurant/customer_interest', Customer_Interest)
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
        smach.State.__init__(self, outcomes=['success', 'failure','preempted'],
                             input_keys=['grasp_ready', 'exist_objects'], 
                             output_keys=['exist_objects'])
        self.object_pub = rospy.Publisher('/restaurant/objects', objects, queue_size=10)
        self.names = None
        functions.HeadAction(0.0, -0.8)

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
        rospy.sleep(5)
        rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.callback)
        rospy.wait_for_message('/darknet_ros/bounding_boxes', BoundingBoxes)
        userdata.exist_objects = self.names
        rospy.loginfo(userdata.exist_objects)
        functions.HeadAction(0.0, 0.0)
        if userdata.grasp_ready:
            # functions.HeadAction(0.0, 0.0)
            return 'preempted'
        else:
            return 'success'



            
class Conversation(smach.State):
    def __init__(self, text):
        smach.State.__init__(self, outcomes=['success', 'failure'],
                             input_keys=['exist_objects'],
                             output_keys=['require_object','grasp_ready'])
        self.text = text

    def execute(self, userdata):
        functions.Speak(self.text)
        rospy.sleep(0.1)
        while not rospy.is_shutdown():
            while not rospy.is_shutdown():
                print(userdata.exist_objects)
                rospy.wait_for_service('/restaurant/wit/listen_interpret')
                service_proxy = rospy.ServiceProxy('/restaurant/wit/listen_interpret', ListenAndInterpret)
                response = service_proxy()
                if response.result != 'nothing':
                    break
                else:
                    functions.Speak('sorry please say again loudly')
            if response.result in userdata.exist_objects:
                functions.Speak('okay ' + response.result + ' please wait for a moment')
                userdata.require_object = response.result
                userdata.grasp_ready = True
                break
            else:
                functions.Speak('sorry we do not have '+ response.result + ' please order something else')

        rospy.loginfo(response.result)
        return 'success'