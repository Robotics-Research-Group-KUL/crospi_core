
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
