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


## Debugging with the etasl_console

An interactive LUA console can be accessed to debug. In order to do this, it is very important to execute the etasl_node by calling `ros2 run` instead of `ros2 launch`, since the terminal needs to "own" the process executed. You can do this by calling:
```bash
ros2 run etasl_ros2 etasl_node
```


Inside the etasl_console you can, for example, print the values of all the expressions defined in the task specification (e.g. inside move_cartesianspace.lua). For printing the expressiongraph type the name of the expression (e.g. `expression_name`) and press enter. To obtain its value use the value() function (e.g. `expression_name:value()`) and press enter.

Try entering `ctx` in the console, which will print information about the solver and task specification. In order to safely exit the console type `quit`.

### Manually entering to the etasl console

To enter to the console manually you can call a ros service. You can do this, for example, by using a different terminal and entering the following command:
```bash
ros2 service call /etasl_node/etasl_console std_srvs/srv/Empty
```


### Automatically entering to the etasl console

You can now also enter the etasl console by specifying a debug monitor in the etasl task specification. This will make the node go to an inactive state first and then enter to the console. An example of such monitor is given below, where the monitor will be triggered after 1 second

```lua
Monitor{
        context=ctx,
        name='finish_and_trigger_console',
        upper=0.0,
        actionname='debug',
        expr=time- constant(1)
}
```

## Authors

- Santiago Iregui <santiago.iregui@kuleuven.be>
- Erwin Aertbeliën <erwin.aertbelien@kuleuven.be>


## TODOs (some...):

- Add Input and Output handlers. 
- Create simrobot node to easily transition between simulation and real robot in the future.
- Implement JSON entry to select which topic fsm/events are published.
- Create ROS param to specify initial joints instead of hardcoding them
-Enable a JSON entry to specify (optionally) the QOS for each input/output handler that subscribes/pubishes to ros topics.Some examples (including one for sensor streaming) are found [here](https://github.com/ros2/rmw/blob/rolling/rmw/include/rmw/qos_profiles.h) and explanation about it [here](https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Quality-of-Service-Settings.html). This is (sort of) equivalent to the connection policy of orocos.
- Change some values of the Schema that Erwin defined as number to integer, to avoid the use of decimals

-fileoutputhandler is not outputing any file for some reason...

-add on_configure to all input and output handlers. Right now I use the initialize in the on_configure but this is confusing and leads to errors.

- I need to change routine that checks which joints are in the task specification and then ignore the rest. I should perhaps publish all joints provided in the list, and the ones not there set as a default value. 

- Implement a parameter in the eTaSL monitors to indicate if the movement should stop immediatly or if the driver should deaccelerate the movement until stopped.

## To debug segmentation fault:

First compile with
```bash
colcon_make --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
colcon_make --packages-select etasl_ros2 --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

Then:

```bash
ros2 run --prefix 'gdb -ex run --args' etasl_ros2 etasl_node --ros-args --params-file param_test.yaml
```

