## Installation

1. Install etasl-cmake libraries following the instructions found [here](https://rob-expressiongraphs.pages.gitlab.kuleuven.be/docker/etasl-cmake/).

2. Clone this repository into your colcon/ament based workspace.

3. Run the following command to create the following alias in your bashrc to source important packages when running or building project using `colcon build`:


```bash
echo "alias ros2_source='source /opt/ros/humble/setup.bash;source /usr/share/colcon_cd/function/colcon_cd.sh;export _colcon_cd_root=/opt/ros/humble/;
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash; source ~/etasl-cmake/standalone-cmake/install/bin/set_etasl_paths.sh'" >> ~/.bashrc
```

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

Note that in ROS2 the devel folder is gone (compared to ROS1), and you need to source in the install directory.