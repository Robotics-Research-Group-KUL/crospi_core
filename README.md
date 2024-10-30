# eTaSL-ROS2 integration

[Under development]


This package was created to be able to develop constraint-based reactive robot behaviors that can fully operate with the ROS2 architecture (without the need of Orocos).


## Dependencies
- ROS2 humble
- colcon


## Documentation

Documentation on how to install and how to use the package is available using MKDOCS. To visualize it, please run the script `documentation/browse.sh` as
```bash
sh documentation/browse.sh
```


<!-- 
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

with this example you should see the robot executing a single task specification, which is specified in the `load_simple_etasl_node.py` launch file. This example can only execute a single task specification, since it is currently lacking integration with a finite state machine or other coordinator (future work). -->


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

