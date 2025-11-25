# Creating Task Specification Libraries

- TODO: Mention to be sure to develop the libraries within an application template, since it contains the code to generate the json schemas automatically
- TODO: Mention conventions of folder to be the library name, but it is not necessary. For example when having multiple versions of the same library, one can change the name of the folders and be able to use both.


This tutorial shows how to configure task specifications and give some suggestions for them to be configurable in favor of reusability. When using a configurable *task specification* coming from a library, it is important to configure it according to the *task* at hand (see Fig. 1). Such configuration is facilitated by the use of JSON Schemas that are **automatically generated**, which provide documentation, autocompletion, validation, among others. This tutorial explains how to make the task specification in such way that the JSON Schemas can be then automatically generated (i.e. from **task specification developer's perspective**).

<figure>
<p align="center" width="100%">
<img src="/images/task_hierarchy_complete.png" alt="init_result" style="width:60%; display: block; margin: 0 auto">
<figcaption><h6 align="center">Fig. 1 - hierarchy diagram of used terms.</h6></figcaption>
</p>
</figure>


In order to make task specifications as generalizable as possible for configuring various tasks, the following elements should be defined within each task specification:

- **Task description:** a brief description of what the task specification does and how to use it.

- **Task parameters:** declaration of a list of parameters used to configure the task execution.

- **Robot model requirements:** declaration of a list of necessary frames that a robot specification must have in order to be compatible with the task specification.

These can be specified by using the `task_requirements` LUA module as shown in the example below. The code will be then explained in chunks. 

```lua linenums="1"
require("context")
require("geometric")
local reqs = require("task_requirements")

local task_description = "This task specification allows to move the position of the end effector in cartesian space relative to the initial pose, while maintaining a constant orientation."

-- ========================================= PARAMETERS ===================================
param = reqs.parameters(task_description,{
    reqs.params.scalar({name="maxvel", description="Maximum velocity [m/s]", default = 0.1, required=true, maximum = 0.5}),
    reqs.params.scalar({name="maxacc", description="Maximum acceleration [m/s^2]", default = 0.1, required=true, maximum = 0.5}),
    reqs.params.string({name="task_frame", description="Name of frame used to control the robot in cartesian space", default = "tcp_frame", required=false}),
    reqs.params.array({name="delta_pos", type=reqs.array_types.number, default={0.0, 0.0, 0.0}, description="3D array of distances [m] that the robot will move w.r.t. the starting position in the X,Y,Z coordinates", required=true,minimum = -1.5, maximum=1.5,minItems = 3, maxItems = 3}),
})

-- ======================================== Robot model requirements ========================================
robot = reqs.robot_model({--This function loads the robot model and checks that all required frames are available
    param.get("task_frame"), --The frame is selected as a parameter, to make the skill even more reusable
    --Add all frames that are required by the task specification
})
robot_joints = robot.robot_joints
task_frame = robot.getFrame(param.get("task_frame"))

-- ========================================= PARAMETERS ===================================
maxvel    = constant(param.get("maxvel"))
maxacc    = constant(param.get("maxacc"))
eqradius  = constant(param.get("eq_r"))

delta_pos = param.get("delta_pos")
delta_x   = constant(delta_pos[1])
delta_y   = constant(delta_pos[2])
delta_z   = constant(delta_pos[3])

...
```

The block of code below is used to first specify the **task description** with a summary of what the task should do. The remaining part of the block defines input **task parameters** that are required to configure an instance of the task specification (i.e. a task). Several task paremeters types (scalar, integer, bool, string, enum, array) can be utilized, and the API for declaring them is included at the end of this document.

```lua linenums="5"
local task_description = "This task specification allows to move the position of the end effector in cartesian space relative to the initial pose, while maintaining a constant orientation."

-- ========================================= PARAMETERS ===================================
param = reqs.parameters(task_description,{
    reqs.params.scalar({name="maxvel", description="Maximum velocity [m/s]", default = 0.1, required=true, maximum = 0.5}),
    reqs.params.scalar({name="maxacc", description="Maximum acceleration [m/s^2]", default = 0.1, required=true, maximum = 0.5}),
    reqs.params.string({name="task_frame", description="Name of frame used to control the robot in cartesian space", default = "tcp_frame", required=false}),
    reqs.params.array({name="delta_pos", type=reqs.array_types.number, default={0.0, 0.0, 0.0}, description="3D array of distances [m] that the robot will move w.r.t. the starting position in the X,Y,Z coordinates", required=true,minimum = -1.5, maximum=1.5,minItems = 3, maxItems = 3}),
})
```

Next in the code we need to declare the **robot model requirements**, which is the necessary information that a robot model needs to be compatible with the current task specification. This part used at configuration time of the tasks (i.e. right before executing the task) to make sure that the robot model contains the necessary information and if so load it. The necessary information that we have considered at the moment is only the list of frames that will be later on constrained or used within the task specification.


```lua linenums="16"
robot = reqs.robot_model({--This function loads the robot model and checks that all required frames are available
    param.get("task_frame"), --The frame is selected as a parameter, to make the skill even more reusable
    --Add all frames that are required by the task specification
})
robot_joints = robot.robot_joints -- Extracts table with robot joint names from the current loaded robot model for later use
task_frame = robot.getFrame(param.get("task_frame")) 
```

In the example above the necessary frame is set as a task parameter such that the same task specification can be configured to deal with multiple frames. For example, in a simple task specification to move in Cartesian space, this could be used to take any arbitrary frame belonging to the robot's kinematic chain to a certain target pose. However, it is also possible to give fixed strings here for less generic tasks.

It is possible to convert the values of the parameters to eTaSL expressions, for instance with the eTaSL function `constant()` as shown below. These parameters could also be evaluated by `if else` statements to for example create constraints in certain situations (e.g. a boolean parameter to create a constraint that makes the robot react compliantly to forces).

```lua linenums="24"
maxvel    = constant(param.get("maxvel"))
maxacc    = constant(param.get("maxacc"))
eqradius  = constant(param.get("eq_r"))

delta_pos = param.get("delta_pos")
delta_x   = constant(delta_pos[1])
delta_y   = constant(delta_pos[2])
delta_z   = constant(delta_pos[3])
```

!!! note
    Use `task parameters` for static parameters that need to be defined at configuration time. These can be assigned as e.g. *constant eTaSL expression* values or can also be for instance evaluated with LUA conditionals (all at configuration time). On the other hand, use [input channels](/markdown/basic_tutorials/subscribing_to_ros_topics) for eTaSL expressions (and only eTaSL expressions) that needs to be updated online (e.g. with inputs of a ROS2 topic). Since the input channels create eTaSL expressions, these cannot be evaluated with LUA conditionals (i.e. `if else`) but need to be evaluated at runtime with eTaSL conditionals (`conditional()` function).



## eTaSL parameters API Documentation

The arguments of the functions shown below, which are used to define all parameters, are organized inside a single LUA table.

### scalar(spec)

Defines a scalar parameter.

#### Arguments
- **`spec`** *(table)*: A table containing the specification of the parameter with specific keys and values:
    - `name` *(string)*: The name of the specified parameter. **Required.**
    - `description` *(string)*: The description of the specified parameter, which serves for documentation. **Required.**
    - `default` *(number)*: The default value of the specified parameter. **Required.**
    - `required` *(boolean)*: Indicates if the parameter is required. Defaults to `true` if not specified. **Optional.**
    - `minimum` *(number)*: Minimum value allowed. **Optional.**
    - `maximum` *(number)*: Maximum value allowed. **Optional.**

#### Returns
- **`param_schema`** *(table)*: A table containing a JSON schema defining the parameter.

---

### int(spec)

Defines an integer parameter.

#### Arguments
- **`spec`** *(table)*: A table containing the specification of the parameter with specific keys and values:
    - `name` *(string)*: The name of the specified parameter. **Required.**
    - `description` *(string)*: The description of the specified parameter, which serves for documentation. **Required.**
    - `default` *(number)*: The default value of the specified parameter. **Required.**
    - `required` *(boolean)*: Indicates if the parameter is required. Defaults to `true` if not specified. **Optional.**
    - `minimum` *(number)*: Minimum value allowed. **Optional.**
    - `maximum` *(number)*: Maximum value allowed. **Optional.**

#### Returns
- **`param_schema`** *(table)*: A table containing a JSON schema defining the parameter.

---

### bool(spec)

Defines a boolean parameter.

#### Arguments
- **`spec`** *(table)*: A table containing the specification of the parameter with specific keys and values:
    - `name` *(string)*: The name of the specified parameter. **Required.**
    - `description` *(string)*: The description of the specified parameter, which serves for documentation. **Required.**
    - `default` *(boolean)*: The default value of the specified parameter. **Required.**
    - `required` *(boolean)*: Indicates if the parameter is required. Defaults to `true` if not specified. **Optional.**

#### Returns
- **`param_schema`** *(table)*: A table containing a JSON schema defining the parameter.

---

### string(spec)

Defines a string parameter.

#### Arguments
- **`spec`** *(table)*: A table containing the specification of the parameter with specific keys and values:
    - `name` *(string)*: The name of the specified parameter. **Required.**
    - `description` *(string)*: The description of the specified parameter, which serves for documentation. **Required.**
    - `default` *(string)*: The default value of the specified parameter. **Required.**
    - `required` *(boolean)*: Indicates if the parameter is required. Defaults to `true` if not specified. **Optional.**
    - `pattern` *(string)*: Indicates if the string should follow a regular expression (regex) pattern. **Optional.**

#### Returns
- **`param_schema`** *(table)*: A table containing a JSON schema defining the parameter.

---

### enum(spec)

Defines an enum parameter.

#### Arguments
- **`spec`** *(table)*: A table containing the specification of the parameter with specific keys and values:
    - `name` *(string)*: The name of the specified parameter. **Required.**
    - `description` *(string)*: The description of the specified parameter, which serves for documentation. **Required.**
    - `default` *(type)*: The default value of the specified parameter. Must belong to `accepted_vals` and correspond to the specified type. **Required.**
    - `required` *(boolean)*: Indicates if the parameter is required. Defaults to `true` if not specified. **Optional.**
    - `type` *(string)*: String specifying the type of enum. Allowed types are `number`, `integer`, and `string`. **Required.**
    - `accepted_vals` *(table)*: A table containing the accepted values of the enum. **Required.**

#### Returns
- **`param_schema`** *(table)*: A table containing a JSON schema defining the parameter.

---

### array(spec)

Defines an array parameter.

#### Arguments
- **`spec`** *(table)*: A table containing the specification of the parameter with specific keys and values:
    - `name` *(string)*: The name of the specified parameter. **Required.**
    - `description` *(string)*: The description of the specified parameter, which serves for documentation. **Required.**
    - `default` *(type)*: The default value of the specified parameter. Must belong to `accepted_vals` and correspond to the specified type. **Required.**
    - `required` *(boolean)*: Indicates if the parameter is required. Defaults to `true` if not specified. **Optional.**
    - `type` *(string)*: String specifying the type of enum. Allowed types are `number`, `integer`, and `string`. **Required.**
    - `minimum` *(number)*: Minimum value allowed. **Optional.**
    - `maximum` *(number)*: Maximum value allowed. **Optional.**
    - `minItems` *(integer)*: Minimum number of items allowed. **Optional.**
    - `maxItems` *(integer)*: Maximum number of items allowed. **Optional.**

#### Returns
- **`param_schema`** *(table)*: A table containing a JSON schema defining the parameter.
