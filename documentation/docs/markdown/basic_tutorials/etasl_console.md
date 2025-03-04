# Debugging eTaSL applications in ROS2

## Introduction

There are several debugging tools that have been developed for the ROS2 community (e.g. ROS bags, bash commands to query data, etc). Thus, in this tutorial we assume that the reader is familiar with those tools and focus on an additional debugging tool that was developed specifically for eTaSL: the *etasl_console*.


The etasl_console is an interactive LUA console can be accessed to visualize properties of the task that was currently being executed and to query variable values. In order to do this, it is very important to execute the etasl_node by calling `ros2 run` instead of `ros2 launch`, since the terminal needs to "own" the process executed. You can do this by calling:
```bash
ros2 run etasl_ros2 etasl_node
```

!!! warning
    If the etasl_node is executed via `ros2 launch` and the etasl_console is executed, the user will not be able to interactively execute commands. Instead, the user will only see a logged message in the terminal informing that the etasl_console was executed, without any explicit error.   



### Manually entering to the etasl console

To enter to the console manually you can call a ros service. You can do this, for example, by using a different terminal and entering the following command:
```bash
ros2 service call /etasl_node/etasl_console std_srvs/srv/Empty
```


### Automatically entering to the etasl console

You can now also enter the etasl console by specifying a debug monitor (i.e. a monitor whose 'actionname' has a string value of 'debug') in the etasl task specification (i.e. the lua file that defines the task that should be debugged). This will make the node go to an inactive state first and then enter to the etasl console. 

!!! example "Example: entering the etasl console with a time monitor"

    This example shows a time monitor that will triggered after 1 second and will run the etasl console.

    ```lua
    Monitor{
        context=ctx,
        name='finish_and_trigger_console',
        upper=0.0,
        actionname='debug',
        expr=time - constant(1)
    }
    ```



## Debugging with the etasl_console

In the terminal that owns the etasl_console you can, for example, print the values of all the expressions defined in the task specification (e.g. inside move_cartesianspace.lua). For printing the expressiongraph type the name of the expression (e.g. `expression_name`) and press enter. To obtain its value use the value() function (e.g. `expression_name:value()`) and press enter.

Try entering `ctx` in the console, which will print information about the solver and task specification. In order to safely exit the console type `quit`.

!!! example

    TODO: This is how we can display an example