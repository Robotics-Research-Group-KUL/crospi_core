#!/usr/bin/env python3

# Copyright (C) 2023  Miguel Ángel González Santamarta

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


import rclpy

from simple_node import Node

from yasmin import CbState
from yasmin import Blackboard
from yasmin import StateMachine
from yasmin import State

from yasmin_ros import ServiceState
from yasmin_ros import MonitorState
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_viewer import YasminViewerPub

from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition

# from lifecycle_msgs.srv import ChangeState_Response

from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data

from etasl_interfaces.srv import TaskSpecificationFile
from etasl_interfaces.srv import TaskSpecificationString

from functools import partial
from colorama import Fore, Back, Style


import etasl_yasmin_utils as etasl
from etasl_state import EtaslState

import time

class Configured(State):
    def __init__(self, sm_node: Node) -> None:
        self.node = sm_node
        super().__init__([SUCCEED,])

    def execute(self, blackboard: Blackboard) -> str:
        print(Style.BRIGHT + Fore.GREEN + 'ENTERING STATE CONFIGURED' + Style.RESET_ALL) #EtaslState does this automatically
        etasl.deactivate(self.node)
        etasl.cleanup(self.node)
        #
        #
        # Here is where we will add the etasl ports and link them with topics of other components
        #
        #
        # time.sleep(1)
        print(Style.BRIGHT + Fore.RED + 'EXITING STATE CONFIGURED' + Style.RESET_ALL) #EtaslState does this automatically

        return SUCCEED
    
class MovingCartesian(EtaslState):
    def __init__(self, node: Node) -> None:
        super().__init__(node=node,  # node
                         topic_name="fsm/events",  # topic name
                         outcomes=["e_finished@etasl_node",],  # explicitly list the events that can be received through the topic. Events that are not specified are ignored
                         entry_handler = self.entry_handler,  # entry handler callback, called once when entering
                         monitor_handler = None,  # monitor handler callback, called several times. If omitted or set to None, the default behavior is to match the topic msg to the outcome
                         exit_handler = self.exit_handler,  # exit handler callback, called once when exiting
                         state_name = "MovingCartesian", #If omitted or set to None, no printing in colors when entering/exiting state
                         )
    
    def entry_handler(self, blackboard: Blackboard):
        etasl.deactivate(self.node)
        etasl.cleanup(self.node)
        etasl.readTaskSpecificationFile(blackboard=blackboard, node=self.node,file_name= "move_cartesianspace.lua", rel_shared_dir=True)
        etasl.configure(self.node)
        etasl.activate(self.node)
        return

    def exit_handler(self, blackboard: Blackboard):
        self.node.get_logger().info("exit handler called")
        return
        # time.sleep(1)
    

class MovingHome(EtaslState):
    def __init__(self, node: Node) -> None:
        super().__init__(node=node,  # node
                         topic_name="fsm/events",  # topic name
                         outcomes=["e_finished@etasl_node",],  # explicitly list the events that can be received through the topic. Events that are not specified are ignored
                         entry_handler = self.entry_handler,  # entry handler callback, called once when entering
                         monitor_handler = None,  # monitor handler callback, called several times. If omitted or set to None, the default behavior is to match the topic msg to the outcome
                         exit_handler = self.exit_handler,  # exit handler callback, called once when exiting
                         state_name = "MovingHome", #If omitted or set to None, no printing in colors when entering/exiting state
                         )
    
    def entry_handler(self, blackboard: Blackboard):
        etasl.deactivate(self.node)
        etasl.cleanup(self.node)
        etasl.readTaskSpecificationFile(blackboard=blackboard, node=self.node,file_name= "move_jointspace_trap.lua", rel_shared_dir=True)
        print("read task spacecification... waiting...")
        # time.sleep(5)
        etasl.configure(self.node)
        print("configured... waiting...")
        # time.sleep(5)
        etasl.activate(self.node)
        print("activated... waiting...")
        # time.sleep(5)

        return

    def exit_handler(self, blackboard: Blackboard):
        self.node.get_logger().info("exit handler called")
        return
        # time.sleep(1)

class MovingJoystick(EtaslState):
    def __init__(self, node: Node) -> None:
        super().__init__(node=node,  # node
                         topic_name="fsm/events",  # topic name
                         outcomes=["e_finished@etasl_node",],  # explicitly list the events that can be received through the topic. Events that are not specified are ignored
                         entry_handler = self.entry_handler,  # entry handler callback, called once when entering
                         monitor_handler = None,  # monitor handler callback, called several times. If omitted or set to None, the default behavior is to match the topic msg to the outcome
                         exit_handler = self.exit_handler,  # exit handler callback, called once when exiting
                         state_name = "MovingJoystick", #If omitted or set to None, no printing in colors when entering/exiting state
                         )
    
    def entry_handler(self, blackboard: Blackboard):
        etasl.deactivate(self.node)
        etasl.cleanup(self.node)
        etasl.readTaskSpecificationFile(blackboard=blackboard, node=self.node,file_name= "move_joystick.lua", rel_shared_dir=True)
        print("read task spacecification... waiting...")
        # time.sleep(5)
        etasl.configure(self.node)
        print("configured... waiting...")
        # time.sleep(5)
        etasl.activate(self.node)
        print("activated... waiting...")
        # time.sleep(5)
        return

    def exit_handler(self, blackboard: Blackboard):
        self.node.get_logger().info("exit handler called")
        return
        # time.sleep(1)


def test_callback(blackboard: Blackboard) -> str:
    blackboard.a = 10
    blackboard.b = 5
    time.sleep(2)
    return SUCCEED


class EtaslFSMNode(Node):


    def __init__(self):
        super().__init__("yasmin_node")
        self.etasl_clients = {} #used to add service clients on the fly

        task_spec_cb = partial(etasl.readTaskSpecificationFile, node=self,file_name= "move_cartesianspace.lua", rel_shared_dir=True)
        
        sm_out = StateMachine(outcomes=["finished_outer"])
        
        sm_out.add_state("CONFIGURED", Configured(self),
                     transitions={SUCCEED: "MovingHome"})

        sm_out.add_state("MovingHome", MovingHome(self),
                     transitions={"e_finished@etasl_node": "MovingCartesian"})
        
        sm_out.add_state("MovingCartesian", MovingCartesian(self),
                     transitions={"e_finished@etasl_node": "MovingJoystick"})
        
        sm_out.add_state("MovingJoystick", MovingJoystick(self),
                     transitions={"e_finished@etasl_node": "STATE_OUTER_B"})
        
        sm_out.add_state("STATE_OUTER_B", CbState([SUCCEED], test_callback),
                     transitions={SUCCEED: "MovingCartesian",
                                ABORT: "finished_outer"})


        # pub
        # YasminViewerPub(self, "Lifecycle FSM", sm)
        YasminViewerPub(self, "Complete FSM", sm_out)

        etasl.define_services(self) #Creates all the etasl service clients and adds them to self.etasl_clients 

        # execute
        outcome = sm_out() #This function will block until the output of sm_out is achieved
        print(outcome)
    
    def add_client(self, name: str, service_type):
        self.etasl_clients[name] = self.create_client(service_type, 'etasl_node/{}'.format(name))
        self.get_logger().info("adding service: etasl_node/{}".format(name))
        return
    
    def call_service(self, srv_name: str, req_task):
        
        return self.etasl_clients[srv_name].call(req_task)


# main
def main(args=None):

    # print("yasmin_etasl")
    rclpy.init(args=args)
    node = EtaslFSMNode()
    node.join_spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
