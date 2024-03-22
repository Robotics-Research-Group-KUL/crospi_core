import smach
# import smach_ros
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
import rclpy
import time
from smach_ros import ServiceState



class State1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])

    def execute(self, userdata):
        # State 1 behavior
        print("state1")
        time.sleep(3)
        return 'outcome1'

class State2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome2'])

    def execute(self, userdata):
        # State 2 behavior
        print("state2")
        time.sleep(3)
        return 'outcome2'
        

def main():
    # Create a StateMachine
    # rclpy.init()

    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])

    # Add states to the StateMachine
    with sm:
        # conf_msg = Transition(id=Transition.TRANSITION_CONFIGURE, label='configure')
        def conf_request_cb (userdata,request):
            conf_request = ChangeState().Request  
            conf_request.id=Transition.TRANSITION_CONFIGURE
            conf_request.label='configure'
            return conf_request

        smach.StateMachine.add('STATE1', State1(), transitions={'outcome1':'STATE2'})
        smach.StateMachine.add('STATE2', State2(), transitions={'outcome2':'CONFIGURE_ETASL'})
        smach.StateMachine.add('CONFIGURE_ETASL', 
            ServiceState('/etasl_node/change_state', 
                ChangeState, 
                request = conf_request_cb),
            transitions={'succeeded':'done'})

    # Execute the StateMachine
    outcome = sm.execute()
    print("Final Outcome:", outcome)

if __name__ == '__main__':
    main()