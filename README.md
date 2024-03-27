# eTaSL-ROS2 integration

[Under development]


This package was created to be able to develop constraint-based reactive robot behaviors that can fully operate with the ROS2 architecture (without the need of Orocos).


## Dependencies
- ROS2 humble
- colcon

## Recommendations

Create the following alias in your bashrc to source important packages when running or building project using `colcon build`:


```bash
alias ros2_source='source /opt/ros/humble/setup.bash;source /usr/share/colcon_cd/function/colcon_cd.sh;export _colcon_cd_root=/opt/ros/humble/;
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash; source ~/etasl-cmake/standalone-cmake/install/bin/set_etasl_paths.sh'
```

## Installation

1. Install etasl-cmake libraries following the instructions found [here](https://gitlab.kuleuven.be/rob-expressiongraphs/docker/etasl-cmake).

2. Clone this repository into your colcon/ament based workspace.

3. run the `ros2_source` alias created above, or run the commands manually on the terminal. This is important as this workspace will be an underlay of the corresponding ros-related sourced workspaces.

4. Compile and install packages by running:

 ```bash 
 colcon build --symlink-install
 ``` 

The flag `--symlink-install` is optional, but highly recommended. Without it you would need to build the package each time that you make any modification to interpreted code (e.g. eTaSL task specifications based on LUA). With this option, colcon installs the relevant files in the shared installation folder by created symbolic links instead of copying the files. 

5. Source the workspace by running:
```bash
 source install/setup.bash
```

Note that in ROS2 the devel folder is gone, and you need to source in the install directory.

## Executing a simple eTaSL node

This node allows to quickly test an eTaSL task specification file, without the hassle of changing your application before testing. The node will execute a task specification and start publishing the solution right away, as soon as it is launched. 

In every terminal that you open first run the `ros2_source` command (alias created above) and then source the workspace (`source install/setup.bash`)

1. In one terminal run 

```bash
ros2 launch etasl_ros2 load_ur10_setup.py
```

to deploy RVIZ with a UR10 robot configuration.

2. In a second terminal run 

```bash
ros2 launch etasl_ros2 load_simple_etasl_node.py
```

with this example you should see the robot executing a single task specification, which is specified in the `load_simple_etasl_node.py` launch file. This example can only execute a single task specification, since it is currently lacking integration with a finite state machine or other coordinator (future work).

## Executing a simple eTaSL lifecycle node

This node allows to quickly test an eTaSL task specification file, without the hassle of changing your application before testing. In contrast to the simple etasl node, this is implemented as a lifecycle node and allows you to easily make changes to the task specification file and executing it without re-launching the node. This node will only run periodically the solver during the active state.

In every terminal that you open first run the `ros2_source` command (alias created above) and then source the workspace (`source install/setup.bash`)

1. In one terminal run 

```bash
ros2 launch etasl_ros2 load_ur10_setup.py
```

to deploy RVIZ with a UR10 robot configuration.

2. In a second terminal run 

```bash
ros2 launch etasl_ros2 load_etasl_node.py
```

with this example you should see the robot executing a single task specification, which is specified in the `load_etasl_node.py` launch file. This example can only execute a single task specification, since it is currently lacking integration with a finite state machine or other coordinator (future work).


## Debugging

An interactive LUA console can be accessed to debug. In order to do this, it is very important to execute the etasl_node by calling `ros2 run` instead of `ros2 launch`, since the terminal needs to "own" the process executed. You can do this by calling:
```bash
ros2 run etasl_ros2 etasl_node --ros-args --params-file param_test.yaml
```

where `param_test.yaml` contains the parameters of the node. For example this file can contain the following:
```yaml
etasl_node:
    ros__parameters:
        task_specification_file: "/home/santiregui/ros2_ws/src/etasl_ros2/etasl/move_cartesianspace.lua"
        jointnames: ["shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"]
```


## Authors

- Santiago Iregui <santiago.iregui@kuleuven.be>
- Erwin Aertbeliën <erwin.aertbelien@kuleuven.be>


## TODOs (some...):
- Use QoS profile suitable to the communication for control and sensors. Some examples (including one for sensor streaming) are found [here](https://github.com/ros2/rmw/blob/rolling/rmw/include/rmw/qos_profiles.h) and explanation about it [here](https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Quality-of-Service-Settings.html). This is (sort of) equivalent to the connection policy of orocos.
- Add Input and Output handlers. 
- Create simrobot node to easily transition between simulation and real robot in the future.
- Implement services for configuration
- Implement a service to access the etasl_console()
- Implement services to read task specification file and read task specification string