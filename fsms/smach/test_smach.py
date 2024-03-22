import smach
# import smach_ros
# from lifecycle_msgs.srv import ChangeState
import rclpy
import time
# from smach_ros import ServiceState



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

    sm = smach.StateMachine(outcomes=['done'])

    # Add states to the StateMachine
    with sm:
        smach.StateMachine.add('STATE1', State1(), transitions={'outcome1':'STATE2'})
        smach.StateMachine.add('STATE2', State2(), transitions={'outcome2':'done'})

    # Execute the StateMachine
    outcome = sm.execute()
    print("Final Outcome:", outcome)

if __name__ == '__main__':
    main()