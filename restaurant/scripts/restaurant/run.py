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
    sm.userdata.grasp_ready = False
    # Open the container
    with sm:
        
        smach.StateMachine.add('NAV_TO_STOREAGE', states.Navigation([1.0,-1.1]),
                               transitions={'success': 'OBJ_DETECTION',
                                            'failure': 'failure'})
        
        smach.StateMachine.add('OBJ_DETECTION', states.ObjectDetection(),
                               transitions={'success': 'STAND_BY',
                                            'preempted': 'GRASP',
                                            'failure': 'failure'})
        
        smach.StateMachine.add('STAND_BY', states.Navigation([1.0,-1.1]),
                               transitions={'success': 'LOOK_LEFT',
                                            'failure': 'failure'})
        
        smach.StateMachine.add('LOOK_LEFT', states.LookAround(0.7),
                               transitions={'success': 'CALLING_LEFT',
                                            'failure': 'LOOK_RIGHT'})
        
        smach.StateMachine.add('LOOK_RIGHT', states.LookAround(-0.7),
                               transitions={'success': 'CALLING_RIGHT',
                                            'failure': 'LOOK_LEFT'})
        
        smach.StateMachine.add('CALLING_LEFT', states.Calling(hand_detection_node),
                               transitions={'success': 'NAVAGATION_LEFT',
                                            'failure': 'LOOK_RIGHT'})
        
        smach.StateMachine.add('CALLING_RIGHT', states.Calling(hand_detection_node),
                               transitions={'success': 'NAVAGATION_RIGHT',
                                            'failure': 'LOOK_LEFT'})
        
        smach.StateMachine.add('NAVAGATION_LEFT', states.Navigation([1.0,-1.1]),
                               transitions={'success': 'ASK',
                                            'failure': 'failure'})
        
        smach.StateMachine.add('NAVAGATION_RIGHT', states.Navigation([1.0,-1.1]),
                               transitions={'success': 'failure',
                                            'failure': 'failure'})

        smach.StateMachine.add('ASK', states.Say('Hello what can I do for you'),
                               transitions={'success': 'ASK',
                                            'failure': 'failure'})
        
        smach.StateMachine.add('HEAR', states.Hear(),
                               transitions={'success': 'CHECK',
                                            'failure': 'ASK'})
        
        smach.StateMachine.add('CHECK', states.CheckObjExist(),
                               transitions={'success': 'NAV_TO_STOREAGE',
                                            'failure': 'ASK'})
        
        smach.StateMachine.add('CHECK', states.CheckObjExist(),
                               transitions={'success': 'NAV_TO_STOREAGE',
                                            'failure': 'failure'})
        
        smach.StateMachine.add('GRASP', states.Pickup(),
                               transitions={'success': 'NAV_TO_CUSTOMER',
                                            'failure': 'failure'})
        
        smach.StateMachine.add('PLACE', states.Place(),
                               transitions={'success': 'STAND_BY',
                                            'failure': 'failure'})
        

        ###DEBUG
        # smach.StateMachine.add('OBDE', states.ObjectDetection(),
        #                        transitions={'success': 'CK',
        #                                     'failure': 'failure'})
        
        # smach.StateMachine.add('CK', states.CheckObjExist('bottle'),
        #                        transitions={'success': 'failure',
        #                                     'failure': 'failure'})
        
    # Use a introspection for visulize the state machine
    sm.set_initial_state(['NAV_TO_STOREAGE'])
    sis = smach_ros.IntrospectionServer('example_server', sm, '/SM_ROOT')
    sis.start()
    # Execute SMACH plan
    outcome = sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()