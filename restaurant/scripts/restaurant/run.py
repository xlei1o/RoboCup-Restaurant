import rospy
import smach
import smach_ros


from restaurant import states
from hand_detection.hand_detection_node import HandDetection
from wit_ros.wit_node import WitRos

# main
def main():
    rospy.init_node('restaurant_workflow')
    # Create a SMACH state machine  
    sm = smach.StateMachine(outcomes=['succeeded', 'failure', 'preempted'])
    # Define user data for state machine
    sm.userdata.grasp_ready = False

    _standby = rospy.get_param('/fixed_coordinates/stand_by')
    _storeageTable = rospy.get_param('/fixed_coordinates/storage_table')
    _leftTable = rospy.get_param('/fixed_coordinates/table_left')
    _rightTable = rospy.get_param('/fixed_coordinates/table_right')

    WIT_API_KEY = rospy.get_param('/wit_api_key')
    ########## Starting Task Manager #######
    hd = HandDetection()  #hand detection
    hd.listen()

    wr = WitRos(WIT_API_KEY)  # speech recognition
    wr.start()

    # Open the container
    with sm:
        
        # smach.StateMachine.add('NAV_TO_STOREAGE', states.Navigation(_storeageTable),
        #                        transitions={'success': 'OBJ_DETECTION',
        #                                     'failure': 'failure'})
        
        # smach.StateMachine.add('OBJ_DETECTION', states.ObjectDetection(),
        #                        transitions={'success': 'STAND_BY',
        #                                     'preempted': 'GRASP',
        #                                     'failure': 'failure'})
        
        # smach.StateMachine.add('STAND_BY', states.Navigation(_standby),
        #                        transitions={'success': 'LOOK_LEFT',
        #                                     'failure': 'failure'})
        
        # smach.StateMachine.add('LOOK_LEFT', states.LookAround(0.7),
        #                        transitions={'success': 'CALLING_LEFT',
        #                                     'failure': 'LOOK_RIGHT'})
        
        # smach.StateMachine.add('LOOK_RIGHT', states.LookAround(-0.7),
        #                        transitions={'success': 'CALLING_RIGHT',
        #                                     'failure': 'LOOK_LEFT'})
        
        # smach.StateMachine.add('CALLING_LEFT', states.Calling(),
        #                        transitions={'success': 'NAVAGATION_LEFT',
        #                                     'failure': 'LOOK_RIGHT'})
        
        # smach.StateMachine.add('CALLING_RIGHT', states.Calling(),
        #                        transitions={'success': 'NAVAGATION_RIGHT',
        #                                     'failure': 'LOOK_LEFT'})
        
        # smach.StateMachine.add('NAVAGATION_LEFT', states.Navigation(_leftTable),
        #                        transitions={'success': 'ASK',
        #                                     'failure': 'failure'})
        
        # smach.StateMachine.add('NAVAGATION_RIGHT', states.Navigation(_rightTable),
        #                        transitions={'success': 'ASK',
        #                                     'failure': 'failure'})

        # smach.StateMachine.add('ASK', states.Say('Hello what can I do for you'),
        #                        transitions={'success': 'HEAR',
        #                                     'failure': 'failure'})
        
        # smach.StateMachine.add('HEAR', states.Hear(wr),
        #                        transitions={'success': 'CHECK',
        #                                     'failure': 'ASK'})
        
        # smach.StateMachine.add('CHECK', states.CheckObjExist('apple'),
        #                        transitions={'success': 'NAV_TO_STOREAGE',
        #                                     'failure': 'ASK'})
        
        
        # smach.StateMachine.add('GRASP', states.Pickup(),
        #                        transitions={'success': 'NAV_TO_CUSTOMER',
        #                                     'failure': 'failure'})
        
        # smach.StateMachine.add('PLACE', states.Place(),
        #                        transitions={'success': 'STAND_BY',
        #                                     'failure': 'failure'})
        
    # sm.set_initial_state(['NAV_TO_STOREAGE'])
###########################################################DEBUG###############################################################
        # smach.StateMachine.add('CALLING_LEFT', states.Calling(),
        #                        transitions={'success': 'failure',
        #                                     'failure': 'OBDE'})
        # smach.StateMachine.add('OBDE', states.ObjectDetection(),
        #                        transitions={'success': 'CK',
        #                                     'failure': 'failure'})
        
        # smach.StateMachine.add('CK', states.CheckObjExist('bottle'),
        #                        transitions={'success': 'failure',
        #                                     'failure': 'failure'})
        # smach.StateMachine.add('LOOK_LEFT', states.LookAround(_leftTable),
        #                        transitions={'success': 'CALLING_LEFT',
        #                                     'failure': 'failure'})
        # smach.StateMachine.add('LOOK_RIGHT', states.LookAround(_rightTable),
        #                        transitions={'success': 'CALLING_RIGHT',
        #                                     'failure': 'failure'})
        # smach.StateMachine.add('CALLING_LEFT', states.Calling(hd),
        #                        transitions={'success': 'NAVAGATION_LEFT',
        #                                     'failure': 'LOOK_RIGHT'})
        # smach.StateMachine.add('CALLING_RIGHT', states.Calling(hd),
        #                        transitions={'success': 'failure',
        #                                     'failure': 'LOOK_LEFT'})
        # smach.StateMachine.add('NAVAGATION_LEFT', states.Navigation(_leftTable),
        #                        transitions={'success': 'ASK',
        #                                     'failure': 'failure'})
        # smach.StateMachine.add('ASK', states.Say('Hello what can I do for you'),
        #                        transitions={'success': 'CHECK',
        #                                     'failure': 'failure'})
        # smach.StateMachine.add('CHECK', states.CheckObjExist('apple'),
        #                        transitions={'success': 'failure',
        #                                     'failure': 'failure'})
        smach.StateMachine.add('NAV_TO_STOREAGE', states.Navigation(_storeageTable),
                               transitions={'success': 'OBJ_DETECTION',
                                            'failure': 'failure'})
        smach.StateMachine.add('OBJ_DETECTION', states.ObjectDetection(),
                               transitions={'success': 'STAND_BY',
                                            'failure': 'failure'})
        smach.StateMachine.add('STAND_BY', states.Navigation(_standby),
                               transitions={'success': 'ASK',
                                            'failure': 'failure'})
        smach.StateMachine.add('ASK', states.Conversation('Hello what can I do for you'),
                               transitions={'success': 'failure',
                                            'failure': 'failure'})
    # Use a introspection for visulize the state machine
    sm.set_initial_state(['NAV_TO_STOREAGE'])
###########################################################DEBUG###############################################################

    sis = smach_ros.IntrospectionServer('example_server', sm, '/SM_ROOT')
    sis.start()
    # Execute SMACH plan
    outcome = sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()