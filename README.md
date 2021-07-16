# clcbs_ros

ROS wrapper for [CL-CBS](https://github.com/APRIL-ZJU/CL-CBS). Follows the structure and format of [MuSHR Coordination](https://github.com/prl-mushr/mushr_coordination)

## Installing

```bash
cd ~/catkin_ws/src/  
git clone --recurse-submodules https://github.com/arnavthareja/clcbs_ros.git
cd ~/catkin_ws
catkin_make
```

## Running
```bash
roslaunch clcbs_ros clcbs_ros.launch
roslaunch clcbs_ros init_planner.launch
```

## Visualizing with Rviz
```bash
rviz -d ~/catkin_ws/src/clcbs_ros/rviz/clcbs.rviz
```
