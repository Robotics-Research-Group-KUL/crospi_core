# Regular Installation

This tutorial guides you through the necessary steps to install the required dependencies the etasl_ros2 related packages. We will use a standard ROS 2 workspace setup.

Alternatively, for using docker, follow the instructions on the [docker](/markdown/installation/docker/#installation-using-a-docker-container) tutorial.

-----

## 🚀 1. Setup and Dependencies

This section covers the installation of fundamental tools like `pip`, `rosdep`, and the Lua package manager **Luarocks**, along with required Lua libraries.

### A. Install Core Tools and Initialize Rosdep

Run the following commands to install necessary dependencies like `pip`, `rosdep`, and **Luarocks**.

```bash
# Install core tools and initialize rosdep
sudo apt-get install -y pip python3-rosdep luarocks
sudo rosdep init
rosdep update
```

### B. Install Lua Dependencies

Use **Luarocks** to install the required Lua libraries, `dkjson` and `jsonschema`.

```bash
# Install Lua libraries
sudo luarocks install dkjson
sudo luarocks install jsonschema
```

-----

## 🛠️ 2. Installing Boost 1.89.0 (Local Build)

The required packages require a specific version of the **Boost C++ Libraries (1.89.0)** for handling lock-free shared memory communication. We will install this version **locally** to avoid conflicts with your system's default Boost installation.

### A. Download and Extract Boost

We will download the source, extract it, and prepare for building.

```bash
# Navigate to the Downloads directory
mkdir -p ~/Downloads
cd ~/Downloads

# Download Boost 1.89.0 source
wget -O boost_1_89_0.tar.gz 'https://sourceforge.net/projects/boost/files/boost/1.89.0/boost_1_89_0.tar.gz/download'

# Extract the archive
tar -xvzf boost_1_89_0.tar.gz
cd boost_1_89_0/
```

### B. Build and Install Locally

The installation will target a directory within your home folder (`$HOME/.local/lib/boost/1.89.0`).

```bash
# Define the installation directory
BOOST_INSTALL_DIR="$HOME/.local/lib/boost/1.89.0"
mkdir -p "$BOOST_INSTALL_DIR"

# Configure, build, and install Boost
./bootstrap.sh --prefix="$BOOST_INSTALL_DIR"
./b2 -j$(nproc) install
```

### C. Cleanup

Remove the downloaded source files to keep your system tidy.

```bash
# Clean up downloaded files
cd ~/Downloads
rm ~/Downloads/boost_1_89_0.tar.gz
rm -rf ~/Downloads/boost_1_89_0/
```

-----

## 📦 3. Create ROS 2 Workspace and Build

Now, you will set up your ROS 2 workspace, clone the main package, install its dependencies, and build.

### A. Setup Workspace and Clone Required Packages

Navigate to your workspace directory (replace `<workspace_dir>` with the actual path, e.g., `~/ros2_ws`) and clone the following packages.

```bash
# Navigate to your workspace source directory (replace <workspace_dir>)
cd <workspace_dir>/src

# Clone the main package (etasl_wrapper)
git clone git@gitlab.kuleuven.be:rob-expressiongraphs/ros2/etasl_interfaces.git
git clone git@gitlab.kuleuven.be:rob-expressiongraphs/ros2/etasl_ros2_default_plugins.git 
git clone --recursive git@gitlab.kuleuven.be:rob-expressiongraphs/ros2/etasl_wrapper.git
git clone --recursive git@gitlab.kuleuven.be:rob-expressiongraphs/ros2/etasl_ros2.git 
git clone --recursive git@gitlab.kuleuven.be:rob-expressiongraphs/ros2/betfsm.git
git clone --recursive git@gitlab.kuleuven.be:rob-expressiongraphs/ros2/etasl_ros2_application_template.git 
```

### B. Resolve Dependencies and Build

Source your ROS 2 environment (assuming **ROS 2 Humble** is installed) and use `rosdep` to install any remaining system dependencies specified in the packages.

```bash
cd <workspace_dir>
# Source your ROS 2 installation (e.g., Humble)
source /opt/ros/humble/setup.bash

# Install dependencies for packages in the 'src' folder
rosdep install --from-paths src -y --ignore-src

# Build the workspace
colcon build --symlink-install
```

??? bug "If a boost-related error pops up :material-cursor-default-click:"
    If a boost-related **error** such as the following pops up:

    ```bash
    /home/developer/devel_ws/install/etasl_ros2/include/robot_interfacing_utils/robotdriver.hpp:16:10: fatal error: boost/lockfree/spsc_value.hpp: No such file or directory 16 | #include <boost/lockfree/spsc_value.hpp>
    |          ^~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    compilation terminated.
    gmake[2]: *** [CMakeFiles/etasl_ros2_default_plugins.dir/build.make:76: CMakeFiles/etasl_ros2_default_plugins.dir/src/simulators/simple_kinematic_simulation.cpp.o] Error 1
    gmake[2]: *** Waiting for unfinished jobs....
    gmake[1]: *** [CMakeFiles/Makefile2:137: CMakeFiles/etasl_ros2_default_plugins.dir/all] Error 2
    gmake: *** [Makefile:146: all] Error 2
    ---
    Failed   <<< etasl_ros2_default_plugins [14.3s, exited with code 2]
    ```

    **Solution:** `touch` the CMakeLists.txt of the package giving problems (in this case `etasl_ros2_default_plugins`) and rebuild:

    ```bash
    # Rebuild the full workspace
    touch src/etasl_ros2_default_plugins/CMakeLists.txt
    colcon build --symlink-install
    ```

!!! success
    You have successfully installed all necessary dependencies and packages\! 
    
    Remember to **source** your workspace before using the software:

    ```bash
    source install/setup.bash
    ```



<!-- # Regular Installation

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

If you have access to our internal repositories, switch to the ssh clone alternative: `git clone --recursive git@gitlab.kuleuven.be:rob-expressiongraphs/ros2/etasl_ros2_application_template.git` -->

