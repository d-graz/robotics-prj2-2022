# Second project of the course "Robotics" (A.Y. 2021-2022)

## Students' ID
- 10660258 (Rogora Matteo)
- 10660259 (Grazzani Davide)

---

## Content packages and files description
- `costmap_processing` service to keep track of the robot's path and save it into a png image
- `dynamic_broadcaster` contains a broadcaster to repeat the odometry's topic as a tf transformation
- `ira_laser_tools` package to manage multiple laser scans, for infos refers to its [documentation](https://arxiv.org/abs/1411.1086)
- `path_visualizer` contains script to visualize robot's path in rviz
- `project2` top level package, only contains project's launchfile and rviz config file

---

## TF tree structure:
List of transformations defining the TF tree
```mermaid
    graph LR
    map --> odom                        %% used to correct position errors
    odom --> base_footprint             %% keeps track of the movements
    base_footprint --zero--> base_link  %% the two frames overlap
    base_link --static--> laser_front   %% position of the front laser
    base_link --static--> laser_rear    %% position of the rear laser
```
The `odom -> base_footprint` transformation is udsed for robot's movements, the `map -> odom` transformation is used to correct robot's position

---

## Bags used
- Map created with `robotics1_final.bag` at `-r4` rate
- Map localization done with `robotics2_final.bag` and `robotics3_final.bag` at `-r2` rate

---

## Map creation node
Map is generated using `map_creation.launch` which uses `slam_gmapping` node of the `gmapping` package

---

## How to start-use nodes

- To start the SLAM environment:
```
roslaunch project2 map_creation.launch
```

- To start the localization environment
(set optinoal parameter `view_path` to enable path viewing in rviz
**WARNING**: this option is disabled by default since enabling it at high bag rates compromises computation):
```
roslaunch project2 robot_localization.launch [view_path:=true]
```

- To save the map generated during mapping and localization as a png image use the service:
```
rosrun costmap_processing path_saver.py <path/image_name.png>
```

---

## Additional infos:
- To run localization process a `map_server` must be already running and pubblishing a map
- Map coordinates in the PNGs appears rotated, so the map image result will appear as rotated, this won't affect path drawing on the map
- Mapping has been performed with `-r4` bag parameter
- Localization has been performed wit `-r2` bag parameter