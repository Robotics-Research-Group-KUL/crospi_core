# Running the eTaSL lifecycle node

!!! abstract

    This tutorial shows how to execute the etasl_node which is part of the etasl_ros2 package. This node is a [lifecycle node](https://design.ros2.org/articles/node_lifecycle.html), which means that the node is not always active but its state needs to be changed after the node is launched. This tutorial only shows how to properly launch the node in an unconfigured state, and how to change its state is left for the [starting a task](/online_configuration/starting_a_task/#starting-a-task) tutorial.


In every terminal that you open first run the `ros2_source` command (alias created above) and then source the workspace (`source install/setup.bash`)

!!! note

    The alias was created during the installation. If this step was ommitted, you can create it with the following command which will include the alias in the `.bashrc`.

    ```bash
        echo "alias ros2_source='source /opt/ros/humble/setup.bash;source /usr/share/colcon_cd/function/colcon_cd.sh;export _colcon_cd_root=/opt/ros/humble/;
        source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash; source ~/etasl-cmake/standalone-cmake/install/bin/set_etasl_paths.sh'" >> ~/.bashrc
    ```


1. In one terminal run the launch file that will execute all your nodes, including RVIZ visualization. 

    ```bash
    ros2 launch <your_package> <your_launchfile>.launch.py
    ```
    !!! example "Example 1: launching UR10 in RVIZ"
        For this example we will execute the following launch file 

        ```bash
        ros2 launch etasl_ros2_application_template load_ur10_setup.launch.py
        ```

        to deploy RVIZ with a UR10 robot configuration. This launch file is available in the etasl_ros2_application_template (see [this tutorial](/template/application_template/#application-template))

2. In a second terminal run:

    ```bash
    ros2 run etasl_ros2 etasl_node --ros-args -p config_file:=/path/to/config_file
    ```

    which will run the etasl_node in an unconfigured state. The `config_file` can be set as any absolute path of a [config_file](/offline_configuration/setup_configuration/#setup-configuration). However, to facilitate the specification of this parameter, the directory can be specified relatively to any ros2 package in the (sourced) workspace:

    ```bash
    ros2 run etasl_ros2 etasl_node --ros-args -p config_file:="\$[my_package]/rest/of/the/path/<my_config_file>.json"
    ``` 

    The etasl_node will interpret the text in between `$[text]` as a package name and will use ament functionalities to search for that package. In bash the `\` before the `$[text]` (i.e. `\$[text]`, see command above) is required, since bash otherwise would try to extract the value of an environment variable called `text` instead of interpreting it as a simple string.


    !!! example "Example 2: using a provided config_file"
        This example is compatible with example 1, since it will run a setup with a UR10 in simulation that can move the virtual robot in RVIZ.

        ```bash
        ros2 run etasl_ros2 etasl_node --ros-args -p config_file:="\$[etasl_ros2_application_template]/applications/application_example_ur10_simulation.setup.json"
        ``` 

    !!! warning
        In contrast to ROS1, ROS2 (with ament tools) searches for the shared directory of the package. Therefore, any files that are not inside one of the installed directories in the shared folder, will not be found. For this make sure that the CMakeLists.txt of your package contains the following:

        ```cmake
        install(DIRECTORY <my_directory_to_be_installed>
        DESTINATION share/${PROJECT_NAME})
        ```

        Everytime you create a new file, make sure to build the package with `colcon build --symlink-install`.

!!! tip

    It is also possible to include the etasl_node into the launch file after development, so that all nodes of the application are executed with the same command. However, during development, it is recommended to keep the separation due to the following reasons:

    1. The terminal owns the node's process and thus only the node can be quitted, without killing all the nodes (e.g. RVIZ remains untouched).
    2. The terminal owns the node's process and thus the etasl_console can be launched for debugging (more information in [debugging tutorial](/debugging/etasl_console/#debugging-etasl-applications-in-ros2)) 
    3. Logged info is displayed clearly in the same terminal and there is no need to use visualization tools such as rqt.

