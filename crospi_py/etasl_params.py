#!/usr/bin/env python3

import json

from typing import List


import ament_index_python as aip

import re




def expand_package_ref( pth:str ) -> str:
    """
    expands all occurencies of $[PACKAGENAME] to the ROS2 packages they refer to.

    Parameters:
      pth: string to expand

    Returns:
        Package path expanded/interpolated with ROS2 ament package path
    """
    def lookup(m):
        pkg=m.group(1)
        try:
            pkg_pth = aip.get_package_share_directory(pkg)
        except:
            raise Exception( f"ament did not find package '{pkg}' while expanding '{pth}'")
        return pkg_pth
    return re.sub(r"\$\[(\w+)\]",lookup, pth)

import os    

def expand_env_ref( pth: str ) -> str:
    """
    expands all occurencies of $[ENVVAR] or $ENVVAR to the ROS2 packages they refer to.
    Parameters:
        pth: string to expand
    Returns:
        String with the value of the environment variable expanded/interpolated
    """
    def lookup(m):
        var=m.group(1)
        if var in os.environ:
            value = os.environ[var]
        else:
            raise Exception( f"not find environment variable '{var}' while expanding '{pth}'")
        return value
    pth= re.sub(r"\$\{(\w+)\}",lookup, pth)
    #pth= re.sub(r"\$(\w+)",lookup, pth)
    return pth


def expand_ref(pth: str) -> str:
    """
    Expands both package references and environment variables.
    
    Parameters:
        pth: string to expand
    Returns:
        String with values of environment variables and ament/ros2 package paths expanded/interpolated
    """    
    return expand_package_ref(expand_env_ref(pth))


def load_task_list( json_file_name: str, blackboard: dict) -> None:
    """
    Loads a task list from a file. References to packages and environment variables in the name
    are expanded (using the expand_... functions)

    Parameters:
        json_file_name: 
            json file containing the task list.
        blackboard:
            blackboard into which to load the task list.
    
    Returns:
        None
    """
    with open(expand_ref(json_file_name), 'r') as json_file:
        parameters = json.load(json_file)
        blackboard["tasks"] = parameters["tasks"]



    # Function to find a task by its name
def get_task(blackboard:dict,task_name:str) -> List[str|List]:
    """
    get the default parameters for a task.

    Parameters:
        blackboard: dict
        task_name: name of the task
    
    Returns:
        Parameters of the task
    """
    try:
        tasks=blackboard["tasks"]
    except:
        raise Exception("There is no `tasks` defined in the blackboard, are you sure you loaded a list of tasks?")
    for task in tasks:
        if task.get("name")==task_name:
            if not("task_specification" in task):
                raise Exception("task dictionary should have `task_specification` keyword")
            if not("parameters" in task["task_specification"]):
                raise Exception("task dictionary should have `parameters` keyword withinn the task_specification keyword")
            return task
    raise Exception(f"No task with name {task_name} found in the blackboard")


def get_value_from_path(key_path: str, dictionary: dict) -> any:
    """
    get a value contained in a dictionary from a string path starting by  

    Parameters:
        key_path: string containing path, e.g. "this/is/the/path/where/the/value/is/in/the/dictionary"
        dictionary: dict

    Returns:
        Value from the specified path located in the provided dictionary
    """

    path = key_path.split("/")
    # Traverse the dictionary
    current = dictionary
    for key in path:
        if key in current:
            current = current[key]
        else:
            print(dictionary)
            raise KeyError(f"Key '{key}' from provided path '{key_path}' was not found in the dictionary.")
    
    return current

def get_task_parameters_filled(blackboard:dict, task_name:str) -> dict:
    """
    get a dictionary with the of list parameters. If a parameter is specified as a string path related to the blackboard: '$blackboard/outputs/my_task/my_parameter, the parameter will be filled in with the corresponding value

    Parameters:
        blackboard:
            blackboard into which to load the task list.
        task_name: 
            name of the task from which the parameters will be obtained

    Returns:
        dictionary with list of parameters filled with valid values
    """

    task = get_task(blackboard, task_name)

    params = {}
    for key,value in task['task_specification']['parameters'].items():
        if isinstance(value, str) and value.startswith("$blackboard"):
            # Remove the "$blackboard" prefix and split the path
            path = value.replace("$blackboard/", "")
            params[key]= get_value_from_path(path, blackboard)  
        else:
            params[key] = value
    return params

def get_task_parameters_raw(blackboard:dict, task_name:str) -> dict:
    """
    get a dictionary with the list parameters. If a parameter is specified as a string path related to the blackboard: '$blackboard/outputs/my_task/my_parameter, it will be kept as it is (otherwise use get_task_parameters_filled function).

    Parameters:
        blackboard:
            blackboard into which to load the task list.
        task_name: 
            name of the task from which the parameters will be obtained
    
    Returns:
        dictionary with list of parameters raw, as it was obtained from the json file
    """

    task = get_task(blackboard, task_name)

    params = {}
    for key,value in task['task_specification']['parameters'].items():
        params[key] = value

    return params


def get_robot_specification_for_task(blackboard:dict, task_name:str) -> str:
    """
    get a string indicating the lua robot specification file that will override the default one. Returns empty string if no overriding lua file was specified

    Parameters:
        blackboard:
            blackboard into which to load the task list.
        task_name: 
            name of the task from which the parameters will be obtained

    Returns:
        String indicating the lua robot specification file, empty string if no overriding lua file was specified
    """
    task = get_task(blackboard, task_name)

    if "robot_specification_file" in task and task["robot_specification_file"] and type(task['robot_specification_file']) == str:
        return  task['robot_specification_file']
    else:
        return ""




def add_task_output_param(blackboard:dict, task_name:str, param_name:str, param_val:any) -> None:
    """
    Adds fields to a blackboard dictionary if they don't exist and sets the value. 
    Afterwards calling this function the following is TRUE: blackboard["output_param"][task_name][param_name] == param_val

    Parameters:
        blackboard:
            blackboard dict where the output parameter will be added.
        task_name: 
            name of the task where the output parameter is computed
        param_name:
            name of the output parameter added
        param_val: 
            value of the output parameter added

    Returns:
        None
    """

    keys = ["output_param", task_name, param_name]
    for key in keys[:-1]:  # Traverse until the second-to-last key
        if key not in blackboard or not isinstance(blackboard[key], dict):
            blackboard[key] = {}  # Create a nested dictionary if the key doesn't exist
        blackboard = blackboard[key]
    blackboard[keys[-1]] = param_val  # Set the value at the final key

