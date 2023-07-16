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
    sm.userdata.server_pos = None

    _standby = rospy.get_param('/fixed_coordinates/stand_by')
    _storeageTable = rospy.get_param('/fixed_coordinates/storage_table')
    _leftTable = rospy.get_param('/fixed_coordinates/table_left')
    _rightTable = rospy.get_param('/fixed_coordinates/table_right')
    _preGrasp = rospy.get_param('/pre_grasp')

    # WIT_API_KEY = rospy.get_param('/wit_api_key')
    # ########## Starting Task Manager #######

    # wr = WitRos(WIT_API_KEY)  # speech recognition
    # wr.start()

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
                               transitions={'success': 'NAV_TO_SERVER',
                                            'failure': 'failure'})

        smach.StateMachine.add('NAV_TO_SERVER', states.Navigation(),
                               transitions={'success': 'failure',
                                            'preempted': 'PLACE',
                                            'failure': 'failure'})

        smach.StateMachine.add('PLACE', states.Place(),
                               transitions={'success': 'STAND_BY',
                                            'failure': 'failure'})

        smach.StateMachine.add('AA', states.Pickup(),
                               transitions={'success': 'failure',
                                            'failure': 'failure'})

    # Use a introspection for visulize the state machine
    # sm.set_initial_state(['AA'])
    sis = smach_ros.IntrospectionServer('example_server', sm, '/SM_ROOT')
    sis.start()
    # Execute SMACH plan
    outcome = sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
