# Robot Laser Simulator (ROS)

Simple robot simulator ROS package using Bresenham ray-casting algorithm. Example visualization in RViz can be watched from Youtube (https://www.youtube.com/watch?v=8J_atau72BQ) and can be seen from the GIF below:

![rviz](rviz.gif)

The given map for the simulator (The robot in the GIF above starting on left bottom side of the map):

![map](launch/map.png)

## Features

- ROS (outputs `/scan` and inputs `/cmd_vel`)
- Publishes simulator clock on ROS (so other nodes can adapt to the simulator speed rate)
- Publishes `map`->`base_link` transform. (No noise implemented yet)
- Laser noise (gaussian mu and variance)
- Easy to read. Maybe? :)

## Running

```bash
$ roslaunch robot_laser_simulator start.launch  # => there are modifiable parameters in this file with descriptions
$ rosrun rqt_robot_steering rqt_robot_steering  # => you can use this command for manually controlling the robot
```

## Reference:

- https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm#All_cases