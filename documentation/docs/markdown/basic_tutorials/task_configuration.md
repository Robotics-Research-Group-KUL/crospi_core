# Configuring Tasks

This tutorial shows how to configure task specifications and give some suggestions for them to be configurable in favor of reusability. When using a configurable *task specification* coming from a library, it is important to configure it according to the *task* at hand (see Fig. 1). Such configuration is facilitated by the use of JSON Schemas that are **automatically generated**, which provide documentation, autocompletion, validation, among others. This tutorial explains how to perform such configuration, which are facilitated by the generated JSON Schemas, so that later on skills can be created with these configured tasks. 


<figure>
<p align="center" width="100%">
<img src="/images/task_hierarchy_complete.png" alt="init_result" style="width:60%; display: block; margin: 0 auto">
<figcaption><h6 align="center">Fig. 1 - hierarchy diagram of used terms.</h6></figcaption>
</p>
</figure>


Instances of task specifications can be created by defining a proper configuration of the task parameters. Such configuration can be in a JSON format and aided through the usage of JSON SCHEMAS that are automatically generated (and updated) when building the package that contains the task specifications (i.e. by running `colcon build`).

Fig. 2 shows how easy it is to configure a task from a desired task specification, thanks to the automatically generated JSON schemas. Once the move_cartesianspace task specification is selected in this example (by typing `"is-move_cartesianspace": true`), only the corresponding `file_path` of the LUA code and only the corresponding parameters of the task specification can be filled in. 

<figure>
<p align="center" width="100%">
<img src="/images/vscode_task_parameters_with_schema.gif" alt="init_result" style="width:70%; display: block; margin: 0 auto">
<figcaption><h6 align="center">Fig. 2 - autocompletion of parameters in VS Code.</h6></figcaption>
</p>
</figure>

However, some parameters are sometimes not known before the execution but need to be determined at runtime (e.g. a position detected by a vision system, or some information gathered in a previously executed state). In those cases, we provide a mechanism that can be used to get it from an application *blackboard* as shown below:

```JSON linenums="9"
"parameters": {
    "delta_pos": "$blackboard/output_param/task_generating_output/delta_pos",
    "eq_r": 0.08,
    "maxacc": 0.1,
    "maxvel": 0.1
}
```

The above indicates that the parameter `delta_pos` should be found within the application blackboard (e.g. structured in a dictionary in python) in the specified path. It is important that the path starts with `$blackboard` so that the program interprets it as a path of the blackboard and not as a regular string. A python helper script `etasl_params.py` is provided within the `etasl_ros2` package easily manage the parameters with a blackboard. An example on how to use this script is provided below:


```python
from etasl_ros2_py import etasl_params

blackboard = {} # Empty dictionary to simplify example
task_name = "movingHome"
etasl_params.load_task_list("my/path/to/json_file.json",blackboard) #Loads JSON into the blackboard dictionary

# -------------- The following gets the path of the robot specification LUA file used for the task
robot_file_path = etasl_params.get_robot_specification_for_task(blackboard, task_name)


# -------------- The following gets a dictionary of parameters filled with the paths provided for the blackboard (e.g. "$blackboard/output_param/task_generating_output/delta_pos")
params_filled = etasl_params.get_task_parameters_filled(blackboard,task_name)

# -------------- The following gets a dictionary of raw parameters, i.e. without getting parameters from the provided blackboard paths provided (e.g. "$blackboard/output_param/task_generating_output/delta_pos" will remain being the same string)
params_raw = etasl_params.get_task_parameters_raw(blackboard,task_name)


# -------------- The following gets the file path of the corresponding task specification of the provided task and expands references e.g. $[etasl_ros2_application_template] will be replaced by the path of the ROS2 package 
task = etasl_params.get_task(blackboard,task_name)  
task_specification file_path = task["task_specification"]["file_path"]
```

!!! note
    The blackboard in the example is a simple dictionary. However, depending on the coordination scheme for the application, this blackboard could be protected with locks/mutexes for shared memory between different threads. 