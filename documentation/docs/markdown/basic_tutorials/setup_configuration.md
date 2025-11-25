# Setup Configuration

!!! abstract
    This tutorial explains how to specify the robot setup. This implies the specification of: 1) **initialization properties** that determine how to the feature variables of the task specifications are initialized 2) **solver properties** that influence how the eTaSL optimization problem is solved online, 3) **robot hardware** to use (e.g. UR10 in simulation or with a real robot), 4) **outputs** to enable (e.g. stream joint positions or task-related variables through ROS topics), 5) **inputs** from sensors or other sources to be used in constraints of the task specification (e.g. twist topic streamed by a joystick ros node). We treat all the above as properties specific to a certain robot setup or specific to a certain application, and therefore are only defined once before execution.

## Configuration file

The aforementioned configuration is made in a JSON file that complies with a [JSON Schema](https://json-schema.org/). This JSON Schema immensely facilitates the creation/modification of configuration files since it can be used to provide autocompletion, documentation, validation, etc, which makes the job of the user very easy.

!!! tip
    Usage of an IDE that supports JSON Schema is strongly recommended, as it will facilitate the creation of the JSON configuration file. We recommend **VScode**, which supports JSON Schema out of the box. In VScode you can press `ctrl+space` for autocompletion (see Fig. 1) and it will display only the possible options that comply with the schema. In addition, if the mouse is hovered over any parameter, documentation will be displayed (see Fig. 2).
    <figure>
    <p align="center" width="100%">
    <img src="/images/autocompletion_json_schema_vscode.gif" alt="init_result" style="width:70%; display: block; margin: 0 auto">
    <figcaption><h6 align="center">Fig. 1 - autocompletion in VS Code.</h6></figcaption>
    </p>
    </figure>


    <figure>
    <p align="center" width="100%">
    <img src="/images/documentation_json_schema_vscode.gif" alt="init_result" style="width:70%; display: block; margin: 0 auto">
    <figcaption><h6 align="center">Fig. 2 - documentation by hovering mouse in VS Code.</h6></figcaption>
    </p>
    </figure>

    **note:** the exact content of Fig.1 and Fig.2 might not be updated


<!-- ![Peek recording itself](https://raw.githubusercontent.com/phw/peek/master/data/screenshots/peek-recording-itself.gif){ .image1percent } -->

<!-- !!! example
<figure>
    <img src="https://raw.githubusercontent.com/phw/peek/master/data/screenshots/peek-recording-itself.gif" alt="drawing" width="400" style="display: block; margin: 0 auto" />
    <figcaption>MDN Logo</figcaption>
    </figure> -->

An example of a complete JSON configuration file is given as follows, and afterwards each section of the configuration file is explained separately. 

```json linenums="1"
{
    "$schema": "../schema/blackboard-schema.json",
    "blackboard": {
        "default-etasl": {
            "general": {
                "event_topic": "etasl/events",
                "sample_time": 0.01
            },
            "initializer": {
                "full": true,
                "sample_time": 0.01,
                "duration": 3.0,
                "convergence_criterion": 1E-8,
                "weightfactor": 1E4
            },
            "solver": {
                "is-qpoasessolver": true,
                "nWSR": 100,
                "regularization_factor": 1E-8,
                "cputime": 10
            },
            "robotdriver":{
                "is-simulationrobotdriver": true,
                "periodicity": 0.01,
                "initial_joints": [3.1416, -1.5708, 1.5708, -1.5708, -1.5708, 0]
            },
            "default_robot_specification": {
                "is-inline_robotspecification": true,
                "urdf_path": "$[etasl_ros2_application_template]/robot_description/urdf/ur10/use_case_setup_ur10.urdf",
                "tcp_frame": {
                    "child_link": "tool0",
                    "parent_link": "base_link"
                },
                "additional_frames": [
                    {
                        "name": "forearm",
                        "child_link": "forearm_link",
                        "parent_link": "base_link"
                    }
                ],
                "robot_joints": [
                    "shoulder_pan_joint",
                    "shoulder_lift_joint",
                    "elbow_joint",
                    "wrist_1_joint",
                    "wrist_2_joint",
                    "wrist_3_joint"
                ]
            },
            "outputhandlers": [
                {
                    "is-jointstateoutputhandler": true,
                    "topic-name": "/joint_states"
                },
                {
                    "is-topicoutputhandler" : true,
                    "topic-name" : "/my_topic",
                    "variable-names" : []
                }
            ],
            "inputhandlers": [
                {
                    "is-twistinputhandler": true,
                    "default_twist": {"linear": {"x": 0, "y": 0, "z": 0},"angular": {"x": 0, "y": 0, "z": 0}},
                    "depth": 1,
                    "number_of_tries": 10,
                    "topic-name": "/spacenav/twist",
                    "when_unpublished": "use_default",
                    "varname": "joystick_input"
                }
            ]
        }
    },
    "$defs": {}
}
```

!!! warning
    Specific content of the configuration file is not relevant for this tutorial, since the documentation of all parameters can be found as seen in Fig. 1 thanks to the JSON Schema. In addition, the structure of the configuration file might change in future updates. But do not fear! It will still use a JSON Schema, which will facilitate things for you if it changes. 

In *line 2* the location of the schema is specified:
```json linenums="2"
"$schema": "../../etasl_ros2/scripts/schema/blackboard-schema.json",
```

so that IDEs such as VS Code can identify which JSON Schema to use for validation and aiding the user. Take note that this configuration file is located in the package `etasl_ros2_application_template` (see [this tutorial](/markdown/template/application_template)), which was cloned in the same directory as the `etasl_ros2` package and follows the directory hierarchy shown below:

```
colcon_workspace
│
└───build
└───install
└───log
└───src
    │
    └───etasl_ros2
    │        └───scripts
    │               └───schema
    │                       blackboard-schema.json
    │
    └───etasl_ros2_application_template
                    └───config_files
                            config_ur10_simulation.json

    
```

Other directory hierarchies can be followed, but the `"$schema"` path must be edited accordingly.

## Configuring general properties

Two general properties can be configured: the name of the topic in which string events triggered by etasl monitors are published (`event_topics`) and the `sample_time` (or periodicity) in seconds at which the controller is updated. In this example the `sample_time` is set to 0.01 seconds, i.e. the controller runs at 100 Hz.

```json linenums="5"
"general": {
    "event_topic": "etasl/events",
    "sample_time": 0.01
},
```

!!! warning
    The `sample_time` will be maintained only if the solver manages to solve the instantaneous optimization problem in a time less than the specified `sample_time`, that is, if the amount of constraints and feature variables is not huge. The checking of jitter and timings related to the real-time execution is left for the robot controller, as every robot has different requirements. Only real-time execution will be obtained under a set of conditions, such as a proper real-time kernel, correct network configuration, proper process priority arrangements, etc. 

## Configuring initializer and solver properties

The initializer refers to the routine that runs at the beginning of each task specification. This routine is an optimization problem that finds the initial values of all the specified feature variables according to the constraints and the initial guess value, both which can be changed when declaring the feature variable in the Lua task specification. For beginner users, it is recommended to use the default values.

```json linenums="9"
"initializer": {
    "full": true,
    "sample_time": 0.01,
    "duration": 3.0,
    "convergence_criterion": 1E-8,
    "weightfactor": 1E4
},
```

**note:** The `sample_time` used in the initializer is only simulated, and not actually executed (i.e. the algorithm will run as fast as possible without "sleeping").

Next, we will configure which solver is used and its corresponding properties. Currently, the only solver supported is qpOASES, but in the future more solvers will be supported. 



```json linenums="13"
"solver": {
    "is-qpoasessolver": true,
    "nWSR": 100,
    "regularization_factor": 1E-8,
    "cputime": 10
},
```

!!! tip
    The `regularization_factor` can be increased to smooth the robot behavior when passing through singularities, with the expense of increasing the tracking error. Therefore, it is recommended to always keep it small (< 1E-3). 

## Configuring the Robot

Configuring the robot implies the configuration of two aspects: configuring the **robotdriver** (i.e. selecting the way in which eTaSL will interface with the robot hardware) and configuring the **robot kinematics**, both which should be compatible.

### Robot driver

eTaSL is able to communicate with multiple robot hardware, each using their own APIs and communication protocols. That knowledge is encapsulated into a robot driver, and the user needs to only select the driver based on the desired robot. That needs to be done in the **first property** of the `robotdriver` object, always starting with the keyword `is-`. In this example a simulation driver for kinematic simulation is specified in the line `"is-simulationrobotdriver": true,`, without any communication with real robot hardware. However, robot drivers that communicate with real hardware can be specified such as `"is-kukaiiwarobotdriver": true,`. Depending on the selected driver, following properties will be enabled (such as `initial_joints` for the simulation driver or `ip_address` for a robot driver communicating over Ethernet).


```json linenums="22"
"robotdriver":{
    "is-simulationrobotdriver": true,
    "periodicity": 0.01,
    "initial_joints": [3.1416, -1.5708, 1.5708, -1.5708, -1.5708, 0]
},
```

!!! note
    In this example, a kinematic simulation of a 6 DOFs robot is specified in the line `"is-simulationrobotdriver": true,`. In this context, simulation means that joint velocity setpoints generated by eTaSL are numerically integrated w.r.t. time to determine the joint positions which are given as feedback to eTaSL. This simulation is generic and is compatible with any robot, with any number of DOFs defined based on the number of initial_joints. If a real robot is specified instead, the joint positions are measured directly from the robot encoders and given to eTaSL as position feedback.

### Robot kinematics

On the other hand, the properties related to the robot kinematics are defined under the `default_robot_specification`. The name has the word "default" since it can be overridden during certain task specifications when more advanced tasks need to be executed. However, for most applications, this will be not needed.

There are two ways in which the robot kinematics can be provided, each specified by the following line: `is-inline_robotspecification` and `is-lua_robotspecification`. The most basic one, shown in the example, is the inline robot specification. In it the kinematics can be provided using a URDF file with a corresponding `urdf_path` and the following properties that depend on the chosen URDF file: 

**Option 1 (basic):**

- `tcp_frame`: is the default frame in which constraints cartesian-space constraints will be applied. Task specifications purely in joint-space do not make use of it. 

- `additional_frames`: is a list of frames (i.e. can be more than one) that can be used to impose constraints. For example a forearm or elbow that can never exceed certain height. There is no difference between a frame created here and the `tcp_frame`, apart from `tcp_frame` being the default frame used for most of the task specifications. While the `child_link` and `parent_link` must be existing links within the URDF, the name can be freely chosen and can be referred to within the task specification.

- `robot_joints`: is a subset of the mobile joints (i.e. non-fixed joints) that exist in the URDF. These will be declared as control variables in eTaSL. 

```json linenums="27"
            "default_robot_specification": {
                "is-inline_robotspecification": true,
                "urdf_path": "$[etasl_ros2_application_template]/robot_description/urdf/ur10/use_case_setup_ur10.urdf",
                "tcp_frame": {
                    "child_link": "tool0",
                    "parent_link": "base_link"
                },
                "additional_frames": [
                    {
                        "name": "forearm",
                        "child_link": "forearm_link",
                        "parent_link": "base_link"
                    }
                ],
                "robot_joints": [
                    "shoulder_pan_joint",
                    "shoulder_lift_joint",
                    "elbow_joint",
                    "wrist_1_joint",
                    "wrist_2_joint",
                    "wrist_3_joint"
                ]
            },
```

**Option 2 (advanced):**

The robot kinematics can be also specified in a LUA file, which allows more flexibility and enables more advanced settings.

```json
"default_robot_specification": {
    "is-lua_robotspecification": true,
    "file_path": "$[etasl_ros2_application_template]/etasl/robot_specifications/ur10.etasl.lua",
    "robot_joints": [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint"
    ]
},
```

The LUA file that contains the kinematics would look something like this:

```lua
require("context")
require("geometric")
local ament = require("libamentlua")
local urdfreader=require("urdfreader")

local M = {}

local etasl_application_share_dir = ament.get_package_share_directory("etasl_ros2_application_template")
local xmlstr = urdfreader.loadFile(etasl_application_share_dir .. "/robot_description/urdf/ur10/use_case_setup_ur10.urdf")
local robot_worldmodel = urdfreader.readUrdf(xmlstr,{})
local VL = {}
local frames = robot_worldmodel:getExpressions(VL,ctx,{tcp_frame={'tool0','base_link'}})

M.frames= frames
M.xmlstr = xmlstr
M.robot_worldmodel = robot_worldmodel
M.urdfreader = urdfreader

return M
```

!!! warning
    With the purpose of optimizing computation time, eTaSL will ignore the joints that do not belong to the kinematic chains created from the world frame to the `tcp_frame` and the `additional_frames`. For example, if the `child_link` of the `tcp_frame` is chosen as the `forearm_link`, the joints that are after this link will not be included in the eTaSL optimization problem and therefore will not be able to move. 

!!! tip
    For path string values, an absolute path can be provided. However this is cumbersome and does not work well when sharing the file. Thus, the path can be placed relatively to a ROS2 package as `$[name_of_ros2_package]` that is part of the current workspace, e.g. `$[etasl_ros2_application_template]`. The string will be replaced by the corresponding path by using the ament library.

## I/O Handlers

Values of eTaSL variables can be outputted/inputted in multiple formats through the use of `outputhandlers` and `inputhandlers`. Most of the available output and input handlers will output/input the requested data through/from a ROS2 topic, but this is not necessarily always the case. For example, an output handler can be created to log information in a CSV file or use a different communication protocol different to the ones provided by ROS2, while an input handler can be created to read data from a raw socket. Nevertheless, the recommended approach is to create ROS2 nodes that perform the desired external action and interface through ROS2 topics with eTaSL. The corresponding publishers/subscribers will be created at configuration time dynamically depending on the configuration of the JSON.

In this tutorial the concept is explained through some examples. More details about the available output/input handlers and how to use them are given in the corresponding "Outputting data" and "Inputting data" tutorials. Advanced users can also create their own output/input handlers through some C++ programming following the instructions given in the "I/O handlers" tutorials. 

### Configuring outputs

In this example two output handlers are created. The first one of type `jointstateoutputhandler` (hence `"is-jointstateoutputhandler": true`) publishes the joint values into the topic `/joint_states` so that RVIZ can display the robot. The second one of type `topicoutputhandler` (hence `"is-topicoutputhandler" : true`) takes all scalar variables that are set as output expressions within the LUA task specification and publishes them. This occurs because the list `variable-names` is left empty, otherwise only the specified variables would be published. 


```json linenums="50"
"outputhandlers": [
    {
        "is-jointstateoutputhandler": true,
        "topic-name": "/joint_states"
    },
    {
        "is-topicoutputhandler" : true,
        "topic-name" : "/my_topic",
        "variable-names" : []
    }
],
```

### Configuring inputs

In this example only one input handler is configured, which is of type `twistinputhandler`. This input handler takes an input from a topic of type `geometry_msgs/Twist` and places it a variable that can be referenced within the task specification. There are multiple properties to be specified, some related to the quality of service.

```json linenums="61"
"inputhandlers": [
    {
        "is-twistinputhandler": true,
        "default_twist": {
            "linear": {"x": 0, "y": 0, "z": 0},
            "angular": {"x": 0, "y": 0, "z": 0}
        },
        "depth": 1,
        "number_of_tries": 10,
        "topic-name": "/spacenav/twist",
        "when_unpublished": "use_default",
        "varname": "joystick_input"
    }
]
```

In this example an expression called `joystick_input` can be used within the task specification to, for instance, create some constraints at the end effector based on twist input coming through the `/spacenav/twist` topic. The way of referring to this input within the task specification in LUA is as follows:

```lua
joystick_input   = ctx:createInputChannelTwist("joystick_input")
```

where the generated `joystick_input` variable can be used as any other eTaSL expression.
