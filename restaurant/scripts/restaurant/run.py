import rospy
import smach
import smach_ros
from restaurant import states
# from wit_ros.wit_node import WitRos

# main


def main():
    rospy.init_node('restaurant_workflow')
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded', 'failure', 'preempted'])
    # Define user data for state machine
    sm.userdata.grasp_ready = False
    sm.userdata.exist_objects = None
    sm.userdata.require_object = None
    sm.userdata.serve_pos = None
    sm.userdata.serve_ready = False

    _standby = rospy.get_param('/fixed_coordinates/stand_by')
    _storeageTable = rospy.get_param('/fixed_coordinates/storage_table')
    _leftTable = rospy.get_param('/fixed_coordinates/table_left')
    _rightTable = rospy.get_param('/fixed_coordinates/table_right')

    with sm:
        smach.StateMachine.add('NAV_TO_STOREAGE', states.Navigation(_storeageTable),
                               transitions={'success': 'OBJ_DETECTION',
                                            'failure': 'failure'})

        smach.StateMachine.add('OBJ_DETECTION', states.ObjectDetection(),
                               transitions={'success': 'STAND_BY',
                                            'preempted': 'PICKUP',
                                            'failure': 'failure'})
        smach.StateMachine.add('STAND_BY', states.Navigation(_standby),
                               transitions={'success': 'LOOK_LEFT',
                                            'failure': 'failure'})

        smach.StateMachine.add('LOOK_LEFT', states.LookAround(0.7),
                               transitions={'success': 'CALLING_LEFT',
                                            'failure': 'LOOK_RIGHT'})

        smach.StateMachine.add('LOOK_RIGHT', states.LookAround(-0.7),
                               transitions={'success': 'CALLING_RIGHT',
                                            'failure': 'LOOK_LEFT'})

        smach.StateMachine.add('CALLING_LEFT', states.Calling(),
                               transitions={'success': 'NAVAGATION_LEFT',
                                            'failure': 'LOOK_RIGHT'})

        smach.StateMachine.add('CALLING_RIGHT', states.Calling(),
                               transitions={'success': 'NAVAGATION_RIGHT',
                                            'failure': 'LOOK_LEFT'})

        smach.StateMachine.add('NAVAGATION_LEFT', states.Navigation(_leftTable),
                               transitions={'success': 'CONVERSATION',
                                            'failure': 'failure'})

        smach.StateMachine.add('NAVAGATION_RIGHT', states.Navigation(_rightTable),
                               transitions={'success': 'CONVERSATION',
                                            'failure': 'failure'})

        smach.StateMachine.add('CONVERSATION', states.Conversation('Hello what can I do for you'),
                               transitions={'success': 'NAV_TO_STOREAGE',
                                            'failure': 'failure'})

        smach.StateMachine.add('PICKUP', states.Pickup(),
                               transitions={'success': 'NAV_TO_SERVE',
                                            'failure': 'failure'})

        smach.StateMachine.add('NAV_TO_SERVE', states.Navigation(),
                               transitions={'success': 'failure',
                                            'preempted': 'PLACE',
                                            'failure': 'failure'})

        smach.StateMachine.add('PLACE', states.Place(),
                               transitions={'success': 'STAND_BY',
                                            'failure': 'failure'})

        # smach.StateMachine.add('AA', states.Pickup(),
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
