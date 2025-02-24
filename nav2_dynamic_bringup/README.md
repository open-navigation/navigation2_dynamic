# nav2_dynamic_bringup

The `nav2_dynamic_bringup` package is an example bringup system for launching carla to test navigation in dynamic environment.

To setup a dynamic environment you will need to run the following script

#### Carla Simulation
1. To run carla simulation run the following script in terminal 1
```shell
sudo docker run --privileged --gpus all --net=host -e DISPLAY=:1 carlasim/carla:0.9.13 /bin/bash ./CarlaUE4.sh
```

#### Carla ROS Bridge
2. To run the carla ros bridge, run the script in terminal 2
```shell
ros2 launch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch.py
```

3. To send the command signal to carla, run the script in terminal 3
```shell
ros2 run carla_twist_to_control carla_twist_to_control
```

4. To generate traffic, run the script in terminal 4
```shell
cd /path_to_carla/CARLA_0.9.13/PythonAPI/examples/
python3 generate_traffic.py
```

#### Nav2 stack for navigation
5. To run the navigation, run the script in terminal 5
```shell
ros2 launch nav2_dynamic_bringup navigation_launch
```
--------------

We will see a step by step procedure for installation of Carla 9.13 on Ubuntu 22.04 

#### Procedure for installation of Carla

Follow the link 
https://carla.readthedocs.io/en/docs-preview/build_docker/
 and run the script
```shell
docker pull carlasim/carla:0.9.13
```
Alternative is to download the carla from the following link
https://github.com/carla-simulator/carla/releases/tag/0.9.13 and download  CARLA_0.9.13.tar.gz

While running Carla outside docker dynamic obstacle can be generated using the python script

```shell
cd /path_to_carla/CARLA_0.9.13/PythonAPI/examples/
python3 generate_traffic.py
```

Same file has to be executed to generate dynamic obstacle while running carla inside docker.
The file has to be executed outside the docker.

#### Procedure for installation of carla-ros-bridge

clone the master branch of carla_ros_bridge from 
follow the instructions from https://carla.readthedocs.io/projects/ros-bridge/en/latest/ros_installation_ros2/

```shell
mkdir -p ~/carla-ros-bridge && cd ~/carla-ros-bridge

git clone --recurse-submodules https://github.com/carla-simulator/ros-bridge.git src/ros-bridge

rosdep update
rosdep install --from-paths src --ignore-src -r

colcon build
```

You will require .egg and .whl file. For running in Ubuntu 22.04.
Download the .egg and .whl file from https://github.com/gezp/carla_ros/releases/tag/carla-0.9.13-ubuntu-22.04
As you would require an egg file compatible for python 3.10.

Copy the downloaded (a) carla-0.9.13-cp310-cp310-linux_x86_64.whl and (b) carla-0.9.13-py3.10-linux-x86_64.egg to /path_to_carla/CARLA_0.9.13/PythonAPI/carla/dist/

Install the .whl file
```shell
cd /path_to_carla/CARLA_0.9.13/PythonAPI/carla/dist/
pip3 install carla-0.9.13-cp310-cp310-linux_x86_64.whl 
```

Then you would need to add carla path in your bash file
```shell
export CARLA_ROOT=/path-to-carla/CARLA_0.9.13/
export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla/dist/carla-0.9.13-py3.10-linux-x86_64.egg:$CARLA_ROOT/PythonAPI/carla
```

Then you can source the carla-ros-bridge workspace
```shell
source ./install/setup.bash
```

Run the script to launch the carla-ros-bridge
```shell
ros2 launch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch.py

ros2 run carla_twist_to_control carla_twist_to_control
```