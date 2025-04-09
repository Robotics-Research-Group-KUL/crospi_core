## Installation using a Docker container

This instructions are meant for developers who want to have a shared directory between the docker container and the regular pc. 

### Using VS Code

1. Clone the repository (recursively) 

```bash
git clone --recursive git@gitlab.kuleuven.be:rob-expressiongraphs/ros2/devcontainer_etasl_ros2.git
``` 

If you want to clone only a specific branch (e.g. `devel/ms7`) run the following:

```bash
git clone -b devel/ms7  --recursive git@gitlab.kuleuven.be:rob-expressiongraphs/ros2/devcontainer_etasl_ros2.git
``` 

2.  Open VSCode in the devcontainer_etasl_ros2 folder


3. Click in the bottom left of VSCode the blue square and select reopen in container. If it is your first time using VSCode devcontainers follow this instructions https://code.visualstudio.com/docs/devcontainers/tutorial


4. In a terminal inside the container (e.g. a container inside VS code) run:
 
``` bash
ros2_source
./build.sh
```

The command `ros2_source` is an alias that is already available in the `bashrc` of the container. It basically sources the relevant ROS2 packages.
