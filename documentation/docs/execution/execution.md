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


1. In one terminal run the launch file that will execute all your nodes, including RVIZ visualization. For this example we will execute a the following launch file 

    ```bash
        ros2 launch etasl_ros2_application_template load_ur10_setup.launch.py
    ```

    to deploy RVIZ with a UR10 robot configuration. This launch file is available in the etasl_ros2_application_template (see [this tutorial](/template/application_template/#application-template))

2. In a second terminal run:

    ```bash
        ros2 run etasl_ros2 etasl_node 
    ```

    which will run the etasl_node in an unconfigured state. 

!!! tip

    It is also possible to include the etasl_node into the launch file after development, so that all nodes of the application are executed with the same command. However, during development, it is recommended to keep the separation due to the following reasons:

    1. The terminal owns the node's process and thus only the node can be quitted, without killing all the nodes (e.g. RVIZ remains untouched).
    2. The terminal owns the node's process and thus the etasl_console can be launched for debugging (more information in [debugging tutorial](/debugging/etasl_console/#debugging-etasl-applications-in-ros2)) 
    3. Logged info is displayed clearly in the same terminal and there is no need to use visualization tools such as rqt.

