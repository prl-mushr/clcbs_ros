# clcbs_ros

ROS wrapper for [CL-CBS](https://github.com/APRIL-ZJU/CL-CBS) with some extra functionality like parameter reconfiguration and the ability to drive through multiple waypoints. Follows the structure and format of [MuSHR Coordination](https://github.com/prl-mushr/mushr_coordination).

## Installing

If you don't already have a full installation of ROS, follow the instructions at [https://wiki.ros.org/Installation](https://wiki.ros.org/Installation) to install ROS on your system.

Install OMPL:

```bash
sudo apt-get install libompl-dev
```

Clone this repo and build:

```bash
cd ~/catkin_ws/src/  
git clone --recurse-submodules https://github.com/arnavthareja/clcbs_ros.git
cd ~/catkin_ws
catkin_make
```

If you want to launch the map server in `launch/init_planner.launch`, install the [MuSHR platform](https://mushr.io/tutorials/quickstart/). To disable the map server, edit the `map_server` arg in `launch/init_planner.launch` to have a value of 0.

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

In a new terminal:

```bash
rviz -d ~/catkin_ws/src/clcbs_ros/rviz/clcbs.rviz
```

Make sure to start rviz before launching `init_planner.launch`.

## API

### Subscribed Topics

#### /{car_n's name}/init_pose ([geometry_msgs/PoseStamped](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseStamped.html))

The initial position of car_n. car_n's name is taken from the config file used in both `clcbs_ros.launch` and `init_planner.launch`.

#### /clcbs_ros/obstacles ([geometry_msgs/PoseArray](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseArray.html))

A list of obstacles in the environment to avoid.

#### /clcbs_ros/goals ([clcbs_ros/GoalPoseArray](#clcbs_ros/GoalPoseArray))

The goal poses for the cars to drive to.

### Published Topics

#### /{car_n's name}/waypoints ([geometry_msgs/PoseArray](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseArray.html))

The navigation plan outputted for car_n. car_n's name is taken from the config file used in both `clcbs_ros.launch` and `init_planner.launch`.

#### /{car_n's name}/marker ([visualization_msgs/Marker](http://docs.ros.org/en/api/visualization_msgs/html/msg/Marker.html))

Markers for visualization of car_n's inputted goal poses. car_n's name is taken from the config file used in both `clcbs_ros.launch` and `init_planner.launch`.

#### /clcbs_ros/border ([visualization_msgs/Marker](http://docs.ros.org/en/api/visualization_msgs/html/msg/Marker.html))

The boundary of the environment for visualization.

### Message Definition

#### clcbs_ros/GoalPoseArray

[`std_msgs/Header`](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Header.html) Header  
`float64` scale  
`float64` minx  
`float64` miny  
`float64` maxx  
`float64` maxy  
[`geometry_msgs/PoseArray[]`](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseArray.html) goals 
