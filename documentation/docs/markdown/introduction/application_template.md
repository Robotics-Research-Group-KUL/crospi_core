# Starting a new application

This tutorial explains how to start a new robot application using etasl_ros2 framework. To facilitate this, we provide a useful [application template](https://gitlab.kuleuven.be/rob-expressiongraphs/ros2/etasl_ros2_application_template) that you can use to create a new package that involves the use of eTaSL in ROS2. 

## Creation of the new application package

The steps for using the template are the following:

1. Clone the template in the src folder of your workspace:
```bash
git clone --recursive https://gitlab.kuleuven.be/rob-expressiongraphs/ros2/etasl_ros2_application_template
```

2. Delete the local git repository
```bash
cd etasl_ros2_application_template
rm -rf .git
```
3. Change the name of the template with a custom name with the assistance of an interactive script that will guide you through the process (e.g. it will ask you for the new name):
```bash
bash configure_template.sh
```
4. Initialize a new git repository (optional but recommended):
```bash
git init
git add .
git commit -m "Initial Commit"
```

5. Build the package using colcon
```bash
cd ..
source install/setup.bash
colcon build --symlink-install
```

## Content of the template

TODO
