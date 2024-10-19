## Executing the eTaSL lifecycle node

!!! abstract

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

