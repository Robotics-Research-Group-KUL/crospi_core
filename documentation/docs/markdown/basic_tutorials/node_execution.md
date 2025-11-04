# Running the eTaSL Lifecycle Node

!!! abstract
    This tutorial explains how to execute the `etasl_node`, which is part of the `etasl_ros2` package.  
    This node is a [lifecycle node](https://design.ros2.org/articles/node_lifecycle.html), meaning it is not always active — its state must be transitioned after launching.  
    This tutorial only covers how to **launch** the node in an *unconfigured* state.  
    Instructions on how to change its lifecycle state are available in the [Starting a Task](/markdown/online_configuration/starting_a_task/#starting-a-task) tutorial.


!!! note:

  Right after opening any terminal, before running any other command, remember to source your workspace:
  ```bash
  source install/setup.bash
  ````


## 1. (optional) Execute launch file with RViz

In the first terminal, execute the launch file that starts RVIZ and other relevant nodes that might be required for your application.

```bash
ros2 launch <your_package> <your_launchfile>.launch.py  #(1)
```

1.  :
    💡 Replace `<your_package>` and `<your_launchfile>` with your own package and launch file.

    Example: to launch a UR10 setup in RViz, run:
    ```bash
        ros2 launch etasl_ros2_application_template load_ur10_setup.launch.py
    ```
    This will deploy RViz with a UR10 robot configuration.
    The launch file is provided by the `etasl_ros2_application_template` package (see [this tutorial](/markdown/introduction/application_template/#application-template)).

---

## 2. Run the eTaSL node

In a second terminal, execute the following command:

```bash
ros2 run etasl_ros2 etasl_node --ros-args -p config_file:=/path/to/config_file -p simulation:=true 
# Click the arrow for more explanation (1)
```

1.  :
    The `config_file` parameter should point to any valid [configuration file](/markdown/basic_tutorials/setup_configuration/#configuration-for-an-application-setup).
    You can also use a relative path with respect to any installed ROS 2 package:

    ```bash
    ros2 run etasl_ros2 etasl_node --ros-args -p config_file:="\$[my_package]/rest/of/the/path/<my_config_file>.json" -p simulation:=true 
    ```

    The eTaSL node interprets the text inside `$[package_name]` as a package name and uses ament utilities to locate the corresponding package path.

    > ⚠️ **Important:** In Bash, escape the `$` (as `\$[text]`) to prevent variable substitution.
    > Otherwise, Bash will try to interpret `$[text]` as an environment variable.

    !!! example "Example 2: Using a provided config_file"
        This example runs a UR10 simulation setup compatible with *Example 1*:

        ```bash
        ros2 run etasl_ros2 etasl_node --ros-args -p config_file:="\$[etasl_ros2_application_template]/applications/application_example_ur10_simulation.setup.json"
        ```

This will run the `etasl_node` in an **unconfigured** state. If simulation:=true it will run the simulation environment that was configured in the configuration file. If simulation:=false, it will execute the robotdrivers that were configured in the configuration file.

??? info "Why not adding the etasl_node to a ROS2 launch file during development :material-cursor-default-click:"
    You can include the `etasl_node` in your launch file once your setup is stable, so all nodes run together.
    However, during development, it is recommended to launch it separately for the following reasons:

    1. **Independent control:** You can stop only the `etasl_node` without shutting down all nodes (e.g., RViz stays open).
    2. **Debugging:** Running the node in its own terminal allows the use of the `etasl_console` for debugging (see [Debugging Tutorial](/debugging/etasl_console/#debugging-etasl-applications-in-ros2)).
    3. **Logging clarity:** Output logs are displayed clearly in the same terminal, avoiding the need for visualization tools like `rqt`.


??? warning "If the configuration files are not located in the standard directories of the etasl_ros2_application_template :material-cursor-default-click:"
    In contrast to ROS 1, ROS 2 (using ament) searches for installed files in a package’s `share` directory.
    Files that are not installed will **not be found** at runtime.
    To ensure your configuration files are discoverable, include the following in your `CMakeLists.txt`:

    ```cmake
    install(
    DIRECTORY <my_directory_to_be_installed>
    DESTINATION share/${PROJECT_NAME}
    )
    ```

    This is already done in the etasl_ros2_application_template, so if you follow the standard directories to include the configuration files, you can ignore this.

Every time you add new files (e.g. python or lua code), rebuild your workspace:

```bash
colcon build --symlink-install
```



