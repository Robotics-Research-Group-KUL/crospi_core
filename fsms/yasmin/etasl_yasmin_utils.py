#!/usr/bin/env python3


import rclpy

from simple_node import Node
from yasmin import Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition

# from lifecycle_msgs.srv import ChangeState_Response

from std_msgs.msg import String
from etasl_interfaces.srv import TaskSpecificationFile
from etasl_interfaces.srv import TaskSpecificationString

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
    repl = node.call_service('readTaskSpecificationFile', req_task)
    time.sleep(1)
    print(repl)
    print("readTaskSpecificationFile")

    return repl

def readTaskSpecificationString(blackboard: Blackboard, node: Node, string_p: String):

    req_task = TaskSpecificationString.Request()
    req_task.str = string_p
    repl = node.call_service('readTaskSpecificationString', req_task)
    print(repl)
    print("readTaskSpecificationString")

    return repl

def configure(node: Node):
    req = ChangeState.Request()
    req.transition.id = 1 #I don't think that the id makes any difference
    req.transition.label = "configure"
    repl = node.call_service('change_state', req)
    print(repl)
    print("configuring")
    return repl

def cleanup(node: Node):
    req = ChangeState.Request()
    req.transition.id = 2 #I don't think that the id makes any difference
    req.transition.label = "cleanup"
    repl = node.call_service('change_state', req)
    print("cleanup")

    return repl

def activate(node: Node):
    req = ChangeState.Request()
    req.transition.id = 3 #I don't think that the id makes any difference
    req.transition.label = "activate"
    repl = node.call_service('change_state', req)
    print("activate")

    return repl
    
def deactivate(node: Node):
    print("===============")
    req = ChangeState.Request()
    req.transition.id = 4 #I don't think that the id makes any difference
    req.transition.label = "deactivate"
    repl = node.call_service('change_state', req)
    print("deactivate")

    return repl