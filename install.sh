#!/bin/bash

apt-get install -y pip python3-rosdep
sudo rosdep init
rosdep update
rosdep install --from-paths src -y --ignore-src

sudo apt-get install -y luarocks

sudo luarocks install dkjson
sudo luarocks install jsonschema


# Install new version 1.89 of boost (for lockfree triple buffer)
#This does not eliminate the current version of Boost (safe to do!)
mkdir -p ~/Downloads
cd ~/Downloads
wget -O boost_1_89_0.tar.gz 'https://sourceforge.net/projects/boost/files/boost/1.89.0/boost_1_89_0.tar.gz/download'
tar -xvzf boost_1_89_0.tar.gz
cd boost_1_89_0/
BOOST_INSTALL_DIR="$HOME/.local/lib/boost/1.89.0"
mkdir -p "$BOOST_INSTALL_DIR"

# Install new boost in parallel without changing currently installed BOOST version
# sudo mkdir -p /opt/boost/1.89.0
# sudo chown $USER:$USER /opt/boost/1.89.0 #
# ./bootstrap.sh --prefix=/opt/boost/1.89.0

./bootstrap.sh --prefix="$BOOST_INSTALL_DIR"
./b2 -j$(nproc) install
# Clean downloaded files
cd ~/Downloads
rm ~/Downloads/boost_1_89_0.tar.gz
rm -rf ~/Downloads/boost_1_89_0/


# cd <workspace_dir>/src
# git clone --recursive git@gitlab.kuleuven.be:rob-expressiongraphs/ros2/etasl_wrapper.git
# cd ..
# source /opt/ros/humble/setup.bash
# colcon build --symlink-install

# source install/setup.bash
# git clone git@gitlab.kuleuven.be:rob-expressiongraphs/ros2/etasl_interfaces.git
# git clone git@gitlab.kuleuven.be:rob-expressiongraphs/ros2/etasl_ros2.git 
# git clone git@gitlab.kuleuven.be:rob-expressiongraphs/ros2/etasl_ros2_default_plugins.git 
# git clone --recursive git@gitlab.kuleuven.be:rob-expressiongraphs/ros2/etasl_ros2_application_template.git 
# git clone --recursive git@gitlab.kuleuven.be:rob-expressiongraphs/ros2/betfsm.git

