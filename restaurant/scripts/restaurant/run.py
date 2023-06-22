import rospy
import smach
import smach_ros

from hand_detection.hand_detection_node import HandDetection
from restaurant import states


# main
def main():
    hand_detection_node = HandDetection()
    hand_detection_node.listen()
    rospy.init_node('restaurant_workflow')
    # Create a SMACH state machine  
    sm = smach.StateMachine(outcomes=['succeeded', 'failure', 'preempted'])
    # Define user data for state machine
    sm.userdata.navGoalInd = 1
    # Open the container
    with sm:

        # # Navigation callback
        # def nav_cb(userdata, goal):
        #     navGoal = MoveBaseGoal()
        #     navGoal.target_pose.header.frame_id = "map"
        #     if userdata.navGoalInd == 1:
        #         rospy.loginfo('Navagate to storage table')
        #         waypoint = rospy.get_param('/way_points/storage_table')
        #         userdata.navGoalInd = 2
        #     elif userdata.navGoalInd == 2:
        #         rospy.loginfo('Navagate to table two')
        #         waypoint = rospy.get_param('/way_points/table_two')
        #         userdata.navGoalInd = 1
        #     navGoal.target_pose.pose.position.x = waypoint["x"]
        #     navGoal.target_pose.pose.position.y = waypoint["y"]
        #     navGoal.target_pose.pose.orientation.z = waypoint["z"]
        #     navGoal.target_pose.pose.orientation.w = waypoint["w"]
        #     return navGoal

        
        smach.StateMachine.add('LOOK_LEFT', states.LookAround(0.7),
                               transitions={'success': 'CALLING_LEFT',
                                            'failure': 'LOOK_RIGHT'})
        
        smach.StateMachine.add('LOOK_RIGHT', states.LookAround(-0.7),
                               transitions={'success': 'CALLING_RIGHT',
                                            'failure': 'LOOK_LEFT'})
        
        smach.StateMachine.add('CALLING_LEFT', states.Calling(hand_detection_node),
                               transitions={'success': 'ASK',
                                            'failure': 'LOOK_RIGHT'})
        
        smach.StateMachine.add('CALLING_RIGHT', states.Calling(hand_detection_node),
                               transitions={'success': 'ASK',
                                            'failure': 'LOOK_LEFT'})
        
        smach.StateMachine.add('NAVAGATION', states.Navigation([1.0,-1.1]),
                               transitions={'success': 'failure',
                                            'failure': 'failure'})

        smach.StateMachine.add('ASK', states.Say('Hello what can I do for you'),
                               transitions={'success': 'failure',
                                            'failure': 'failure'})
        

    # Use a introspection for visulize the state machine
    sm.set_initial_state(['LOOK_LEFT'])
    sis = smach_ros.IntrospectionServer('example_server', sm, '/SM_ROOT')
    sis.start()
    # Execute SMACH plan
    outcome = sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()