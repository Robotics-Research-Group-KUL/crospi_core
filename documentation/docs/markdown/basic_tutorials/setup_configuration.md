# Setup/application Configuration

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


<!-- ![Peek recording itself](https://raw.githubusercontent.com/phw/peek/master/data/screenshots/peek-recording-itself.gif){ .image1percent } -->

<!-- !!! example
<figure>
    <img src="https://raw.githubusercontent.com/phw/peek/master/data/screenshots/peek-recording-itself.gif" alt="drawing" width="400" style="display: block; margin: 0 auto" />
    <figcaption>MDN Logo</figcaption>
    </figure> -->

An example of a complete JSON configuration file is given as follows, and afterwards each section of the configuration file is explained separately. 

```json linenums="1"
{
    "$schema": "../../etasl_ros2/scripts/schema/blackboard-schema.json",
    "blackboard": {
        "default-etasl": {
            "sample_time": 0.01,
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
                "initial_joints": [0,0,0,0,0,0],
                "joint_names": ["shoulder_pan_joint",
                                "shoulder_lift_joint",
                                "elbow_joint",
                                "wrist_1_joint",
                                "wrist_2_joint",
                                "wrist_3_joint"]
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

so that IDEs such as VS Code can identify which JSON Schema to use for validation and aiding the user. Take note that this configuration file is located in the package `etasl_ros2_application_template` (see [this tutorial](/template/application_template/#application-template)), which was cloned in the same directory as the `etasl_ros2` package and follows the directory hierarchy shown below:

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

## Configuring initializer and solver properties

Next the sample_time (i.e. periodicity) in which the optimization problem is solved to update the controller is defined in seconds. In this example the sample time is 0.01 seconds, i.e. the controller runs at 100 Hz. Subsequently, properties for the routine that initialize the feature variables can be defined. This routine is an optimization problem that finds the initial values of all the specified feature variables according to the constraints and the initial guess value (which can be changed when declaring the feature variable in Lua). It is recommended to leave the default values, unless you are an advanced user.

```json linenums="5"
"sample_time": 0.01,
"initializer": {
    "full": true,
    "sample_time": 0.01,
    "duration": 3.0,
    "convergence_criterion": 1E-8,
    "weightfactor": 1E4
},
```

!!! warning 
    If the solver takes a longer time to solve the optimization problem that is generated from the task specification, the `sample_time` will be larger than what it is specified (i.e. the controller will run at a lower rate).

Next, which solver is used, and its corresponding properties are defined. Currently, the only solver supported is qpOASES, but in the future more solvers will be supported. 

!!! tip
    The `regularization_factor` can be increased to smooth the robot behavior when passing through singularities, with the expense of increasing the tracking error. Therefore, it is recommended to always keep it small (< 1E-3) 

```json linenums="13"
"solver": {
    "is-qpoasessolver": true,
    "nWSR": 100,
    "regularization_factor": 1E-8,
    "cputime": 10
},
```

## Configuring the Robot

Configuring the robot implies the configuration of two aspects: configuring the **robotdriver** (i.e. selecting the way in which eTaSL will interface with the robot hardware) and configuring the **robot kinematics** (based on a URDF file).

### Robot driver

In the next part of the configuration file, the robot with its corresponding driver is defined. The Schema will only allow to define first the type which begins with `is-`, and all the supported options will be displayed in the IDE.

In this example, a kinematic simulation of a 6 DOFs robot is specified in the line `"is-simulationrobotdriver": true,`. In this context, simulation means that joint velocity setpoints generated by eTaSL are integrated w.r.t. time to determine the joint positions which are given as feedback to eTaSL. This simulation is generic and is compatible with any robot, with any number of DOFs. The only important aspect here is that the `joint_names` should coincide with the controllable joints specified in the URDF file.

If a real robot is specified instead, the joint positions are measured directly from the robot encoders and given to eTaSL as feedback.

```json linenums="19"
"robotdriver":{
    "is-simulationrobotdriver": true,
    "periodicity": 0.01,
    "initial_joints": [0,0,0,0,0,0],
    "joint_names": ["shoulder_pan_joint",
                    "shoulder_lift_joint",
                    "elbow_joint",
                    "wrist_1_joint",
                    "wrist_2_joint",
                    "wrist_3_joint"]
},
```

### Robot kinematics

TODO

## Configuring outputs

```json linenums="30"
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

## Configuring inputs

```json linenums="41"
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
TODO: Explain the keyword `is-` and that the other properties only show up when these keyword is defined. This applies for fields that can have multiple options

TODO: Mention that further explanation on the robot drivers, input handlers and output handlers is given in other tutorials