# clcbs_ros

ROS wrapper for [CL-CBS](https://github.com/APRIL-ZJU/CL-CBS). Follows the structure and format of [MuSHR Coordination](https://github.com/prl-mushr/mushr_coordination)

## Installing

If you don't already have a full installation of ROS, follow the instructions at [https://wiki.ros.org/Installation](https://wiki.ros.org/Installation) to install ROS on your system.

Install OMPL:

```bash
sudo apt-get install libompl-dev
```

If you want to launch the map server in `launch/init_planner.launch`, install the [MuSHR platform](https://mushr.io/tutorials/quickstart/). To disable the map server, edit the `map_server` arg in `launch/init_planner.launch` to have a value of 0.

Clone this repo and build:

```bash
cd ~/catkin_ws/src/  
git clone --recurse-submodules https://github.com/arnavthareja/clcbs_ros.git
cd ~/catkin_ws
catkin_make
```

## Running

Open 3 terminals.

Terminal 1:

```bash
roscore
```

Terminal 2:

```bash
roslaunch clcbs_ros clcbs_ros.launch
```

Terminal 3:

```bash
roslaunch clcbs_ros init_planner.launch
```

## Visualizing with rviz

```bash
rviz -d ~/catkin_ws/src/clcbs_ros/rviz/clcbs.rviz
```

(Make sure to start rviz before launching `init_planner.launch`)
