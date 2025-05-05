# Regular Installation

!!! abstract
    
    This tutorial shows how to perform a regular installation that is compatible with the ROS2 (COLCON/AMENT) ecosystem. Two options are provided for this installation. If the use of docker is desired, skip this tutorial and follow instead the next tutorial which shows how to perform the installation using a provided docker image.

## Option 1 (recommended):

This option consists on using the etasl_wrapper package which wraps etasl-cmake core libraries in a ROS2 ament package so that it can be installed via colcon.

1. Install the etasl core libraries by first installing dependencies. A minimalistic set of commands to install dependencies is provided as follows, which should work in most systems:

```bash
    sudo apt-get update
    sudo apt-get install -y ros-humble-joint-state-publisher-gui luarocks 
    sudo luarocks install dkjson
    sudo luarocks install jsonschema
```

This assumes that the ROS2 full-desktop installation was performed previously.

2. Install the etasl_wrapper package, which wraps etasl-cmake core libraries in a ROS2 ament package so that it can be installed via colcon within a ROS2 workspace:

```bash
    cd <my_ros2_workspace>
    source install/setup.bash
    cd src
    git clone --recursive https://gitlab.kuleuven.be/rob-expressiongraphs/ros2/etasl_wrapper.git
    cd ..
    colcon build --symlink-install
```

The `--symlink-install` flag is recommended but not enforced. 

If you have access to our internal repositories, switch to the ssh clone alternative: `git clone --recursive git@gitlab.kuleuven.be:rob-expressiongraphs/ros2/etasl_wrapper.git`

!!! note

    If the installation above fails because it requires an additional dependency, please cherry-pick the required package from the exhaustive list below.
    ```bash

        sudo apt-get update
        sudo apt-get install -y git wget python3-pip vim tree bash-completion python3-venv\
        lua5.1 liblua5.1.0-dev cython3 ninja-build cmake-curses-gui virtualenv gdb \
        libpugixml-dev doxygen clang-tidy librange-v3-dev xdot psmisc \
        ros-humble-controller-manager \
        ros-humble-position-controllers ros-humble-joint-trajectory-controller ros-humble-control-msgs \
        ros-humble-behaviortree-cpp-v3 libqt5svg5-dev libdw-dev ros-humble-joy-teleop ros-humble-twist-mux \
        ros-humble-launch-param-builder ros-humble-nav2-msgs ros-humble-joint-state-publisher-gui \
        ros-humble-moveit-configs-utils ros-humble-moveit-ros-control-interface ros-humble-moveit-ros-move-group \
        ros-humble-moveit-kinematics ros-humble-moveit-planners-ompl ros-humble-moveit-ros-visualization \
        ros-humble-joint-state-broadcaster ros-humble-diff-drive-controller ros-humble-pal-navigation-cfg-bringup \
        ros-humble-gazebo-plugins ros-humble-gazebo-ros ros-humble-gazebo-ros2-control ros-humble-pal-gazebo-plugins \
        ros-humble-play-motion2-msgs ros-humble-play-motion2 ros-humble-teleop-tools-msgs ros-humble-vision-msgs  \
        ros-humble-ros-testing ros-humble-turtle-tf2-py ros-humble-tf2-tools ros-humble-tf-transformations \
        ros-humble-rqt-reconfigure ros-humble-rqt-plot ros-humble-rqt-topic ros-humble-rqt-tf-tree ros-humble-rqt-moveit \
        ros-humble-rqt-joint-trajectory-controller ros-humble-resource-retriever ros-humble-ur-robot-driver \
        ros-humble-rqt-plot \
        luarocks 

        sudo luarocks install dkjson
        sudo luarocks install jsonschema
    ```

3. Intall the etasl_interfaces package, which contains basic definitions of ROS2 messages/services for interfacing with the etasl_node:

```bash
    cd <my_ros2_workspace>
    source install/setup.bash
    cd src
    git clone --recursive https://gitlab.kuleuven.be/rob-expressiongraphs/ros2/etasl_interfaces.git
    cd ..
    colcon build --symlink-install
```

The `--symlink-install` flag is recommended but not enforced. 

If you have access to our internal repositories, switch to the ssh clone alternative: `git clone --recursive git@gitlab.kuleuven.be:rob-expressiongraphs/ros2/etasl_interfaces.git`

4. Intall the etasl_ros2 package (inside this repository):

```bash
    cd <my_ros2_workspace>
    source install/setup.bash
    cd src
    git clone --recursive https://gitlab.kuleuven.be/rob-expressiongraphs/ros2/etasl_ros2.git
    cd ..
    colcon build --symlink-install
```

The `--symlink-install` flag is recommended but not enforced. 

If you have access to our internal repositories, switch to the ssh clone alternative: `git clone --recursive git@gitlab.kuleuven.be:rob-expressiongraphs/ros2/etasl_ros2.git`


5. Intall the betfsm package, which will allow you to run examples provided in the templates:

```bash
    cd <my_ros2_workspace>
    source install/setup.bash
    cd src
    git clone --recursive https://gitlab.kuleuven.be/rob-expressiongraphs/ros2/betfsm.git
    cd ..
    colcon build --symlink-install
```

The `--symlink-install` flag is recommended but not enforced. 

If you have access to our internal repositories, switch to the ssh clone alternative: `git clone --recursive git@gitlab.kuleuven.be:rob-expressiongraphs/ros2/betfsm.git`

6. Intall the etasl_ros2_application_template package, which contains the examples of applications, and will be a key element in the following tutorials and in development of applications:

```bash
    cd <my_ros2_workspace>
    source install/setup.bash
    cd src
    git clone --recursive https://gitlab.kuleuven.be/rob-expressiongraphs/ros2/etasl_ros2_application_template.git
    cd ..
    colcon build --symlink-install
```

The `--symlink-install` flag is recommended but not enforced. 

If you have access to our internal repositories, switch to the ssh clone alternative: `git clone --recursive git@gitlab.kuleuven.be:rob-expressiongraphs/ros2/etasl_ros2_application_template.git`


## Option 2

This option consists on installing the etasl-cmake core libraries with plain CMAKE or with a debian package instead of colcon.

1. Install etasl-cmake libraries following the instructions found [here](https://rob-expressiongraphs.pages.gitlab.kuleuven.be/docker/etasl-cmake/).

2. Install the etasl core libraries by first installing dependencies. A minimalistic set of commands to install dependencies is provided as follows, which should work in most systems:

```bash
    sudo apt-get update
    sudo apt-get install -y ros-humble-joint-state-publisher-gui luarocks 
    sudo luarocks install dkjson
    sudo luarocks install jsonschema
```

This assumes that the ROS2 full-desktop installation was performed previously.

3. Intall the etasl_interfaces package, which contains basic definitions of ROS2 messages/services for interfacing with the etasl_node:

```bash
    cd <my_ros2_workspace>
    source install/setup.bash
    cd src
    git clone --recursive https://gitlab.kuleuven.be/rob-expressiongraphs/ros2/etasl_interfaces.git
    cd ..
    colcon build --symlink-install
```

The `--symlink-install` flag is recommended but not enforced. 

If you have access to our internal repositories, switch to the ssh clone alternative: `git clone --recursive git@gitlab.kuleuven.be:rob-expressiongraphs/ros2/etasl_interfaces.git`

4. Run the following command to create the following alias in your bashrc to source important packages when running or building project using `colcon build`:


```bash
echo "alias ros2_source='source /opt/ros/humble/setup.bash;source /usr/share/colcon_cd/function/colcon_cd.sh;export _colcon_cd_root=/opt/ros/humble/;
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash; source ~/etasl-cmake/standalone-cmake/install/bin/set_etasl_paths.sh'" >> ~/.bashrc
```

5. Call the previously created alias (`ros2_source`) (this is important as this workspace will be an underlay of the corresponding ros-related sourced workspaces) and intall the etasl_ros2 package (inside this repository):

```bash
    cd <my_ros2_workspace>/src
    git clone --recursive https://gitlab.kuleuven.be/rob-expressiongraphs/ros2/etasl_ros2.git
    cd ..
    colcon build --symlink-install
```

The `--symlink-install` flag is recommended but not enforced. 

If you have access to our internal repositories, switch to the ssh clone alternative: `git clone --recursive git@gitlab.kuleuven.be:rob-expressiongraphs/ros2/etasl_ros2.git`


6. Source the workspace by running:
```bash
 source install/setup.bash
```

7. Intall the betfsm package, which will allow you to run examples provided in the templates:

```bash
    cd <my_ros2_workspace>
    source install/setup.bash
    cd src
    git clone --recursive https://gitlab.kuleuven.be/rob-expressiongraphs/ros2/betfsm.git
    cd ..
    colcon build --symlink-install
```

The `--symlink-install` flag is recommended but not enforced. 

If you have access to our internal repositories, switch to the ssh clone alternative: `git clone --recursive git@gitlab.kuleuven.be:rob-expressiongraphs/ros2/betfsm.git`

8. Intall the etasl_ros2_application_template package, which contains the examples of applications, and will be a key element in the following tutorials and in development of applications:

```bash
    cd <my_ros2_workspace>
    source install/setup.bash
    cd src
    git clone --recursive https://gitlab.kuleuven.be/rob-expressiongraphs/ros2/etasl_ros2_application_template.git
    cd ..
    colcon build --symlink-install
```

The `--symlink-install` flag is recommended but not enforced. 

If you have access to our internal repositories, switch to the ssh clone alternative: `git clone --recursive git@gitlab.kuleuven.be:rob-expressiongraphs/ros2/etasl_ros2_application_template.git`