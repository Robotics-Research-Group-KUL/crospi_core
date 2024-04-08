#!/usr/bin/env python3


import rclpy

from simple_node import Node
from yasmin import Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition

# from lifecycle_msgs.srv import ChangeState_Response

from std_msgs.msg import String
from etasl_ros2.srv import TaskSpecificationFile
from etasl_ros2.srv import TaskSpecificationString

from functools import partial

import time

def define_services(node: Node):
    # node.add_client("configure", ChangeState)
    # node.add_client("cleanup", ChangeState)
    # node.add_client("activate", ChangeState)
    # node.add_client("deactivate", ChangeState)
    node.add_client("change_state", ChangeState)
    node.add_client("readTaskSpecificationFile", TaskSpecificationFile)
    node.add_client("readTaskSpecificationString", TaskSpecificationString)

    #Add all the etasl services here


def readTaskSpecificationFile(blackboard: Blackboard, node: Node, file_name: String, rel_shared_dir: bool):
    req_task = TaskSpecificationFile.Request()
    req_task.file_path = file_name
    req_task.rel_shared_dir = rel_shared_dir
    node.call_service('readTaskSpecificationFile', req_task)
    time.sleep(1)

    return

def readTaskSpecificationString(blackboard: Blackboard, node: Node, string_p: String):

    req_task = TaskSpecificationString.Request()
    req_task.str = string_p
    node.call_service('readTaskSpecificationString', req_task)

    return

def configure(node: Node):
    req = ChangeState.Request()
    req.transition.id = 1 #I don't think that the id makes any difference
    req.transition.label = "configure"
    node.call_service('change_state', req)
    return

def cleanup(node: Node):
    req = ChangeState.Request()
    req.transition.id = 2 #I don't think that the id makes any difference
    req.transition.label = "cleanup"
    node.call_service('change_state', req)
    return

def activate(node: Node):
    req = ChangeState.Request()
    req.transition.id = 3 #I don't think that the id makes any difference
    req.transition.label = "activate"
    node.call_service('change_state', req)
    return
    
def deactivate(node: Node):
    print("===============")
    req = ChangeState.Request()
    req.transition.id = 4 #I don't think that the id makes any difference
    req.transition.label = "deactivate"
    node.call_service('change_state', req)
    return