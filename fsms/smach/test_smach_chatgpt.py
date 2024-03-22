import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState
import smach
import smach_ros

class ChangeStateState(smach.State):
    def __init__(self, node_):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        node_ = node

        # Create a client for the ChangeState service
        self.client = node.create_client(ChangeState, 'lifecycle_node_name/change_state')

    def execute(self, userdata):
        # Wait for the service to become available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('ChangeState service not available, waiting...')

        # Create a request object
        request = ChangeState.Request()
        request.transition.id = ChangeState.Request.TRANSITION_CONFIGURE

        # Call the service
        future = self.client.call_async(request)

        # Handle the response
        try:
            response = future.result()
            self.node.get_logger().info('State changed to configure')
            return 'success'
        except Exception as e:
            self.node.get_logger().error(f'Failed to change state: {e}')
            return 'failure'

def main():
    rclpy.init()
    node = rclpy.create_node('smach_node')

    # Create a StateMachine
    sm = smach.StateMachine(outcomes=['done'])

    # Add states to the StateMachine
    with sm:
        smach.StateMachine.add('CHANGE_STATE', ChangeStateState(node), transitions={'success':'done', 'failure':'CHANGE_STATE'})

    # Execute the StateMachine
    outcome = sm.execute()
    print("Final Outcome:", outcome)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
