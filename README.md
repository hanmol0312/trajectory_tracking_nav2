# trajectory_tracking_nav2

## 1. Introduction

This Repository manages saved trajectories and uses them later for visualizations. Using ROS2 and NAV2 if required for tracking navigation. 

## 2. Clone the repository.

```
mkdir my_ws
cd my_ws
mkdir src
cd src
git clone https://github.com/hanmol0312/trajectory_tracking_nav2.git

```

## 3. Build the project.

```
cd ..
colcon build
```

## 4. Launch the simulation

```
ros2 launch waypoint_navigator robot.launch.xml 
```

- This launch file will also launch the trajectory_saver node that will save the trajectory in a yaml file with the filename as mentioned by the user.

## 5. **Run the trajectory_saver node.(If required)**

```
ros2 run trajectory_op traj_publisher_saver 
```

## 6. Service Call via terminal.

```python
 ros2 service call /Saver tutorial_interfaces/srv/Saver "{duration: 10, file_name: 'trajectory'}"
```

- Can change the duration and file_name as required.
- The file will be saved in the share directory of the package for later use to read the trajectory for visualization.

## 7. Give a goal through nav2.

(![Screenshot from 2025-03-01 10-28-31](https://github.com/user-attachments/assets/8ed9b5b0-4c30-4847-818c-6920e4e81a0a)
)



## 7. Visualization of trajectory.

- Run the node for visualization with file name as the argument.

```python
ros2 run trajectory_op traj_publisher_reader --ros-args -p filename:=trajectory
```

## 8. The trajectory will be published on a topic as a Marker Array.

(![Screenshot from 2025-03-01 10-32-08](https://github.com/user-attachments/assets/f07f0ea0-0e4c-4b5a-9b5b-7439d1225c64)
)


## 9. Approach

- The trajectory saver node will continuously subscribe to the odom callback and if there is a significant change in movement with a distance threshold then the marker will be intialized at that particular position and will be pushed in the array also there will be global counter that will have the count of markers appended.
- Once the request is received by the server the markers from the start_count till the end_count after the time duration will be taken into consideration for loading the trajectory in the yaml file that will be saved in the share of the directory.
- The trajectory_reader node will just read the yaml file and will publish the marker_array. That can be visualized in the rviz as seen in the image above.
