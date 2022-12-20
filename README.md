# Assignment 1

The goal in this assignment is to explore concepts of perception in a robotic system to accomplish a task. Given a mobile robot with a set of sensors in a partially known environment, objects/obstacles must be detected and counted. In addition, the robot must be started in a random position and not rely on any teleoperated commands.

## Methodology

The task to be solved here has been divided into several other tasks that together are the complete assignment resolution.

### Random start

To be able to initialize the robot in a random position, the [`worlds_gazebo`](https://github.com/autonomous-robots/worlds_gazebo) repository was built. When launching, one of the worlds will be randomly chosen as well as the position of the robot. The [python-sdformat](https://pypi.org/project/python-sdformat/0.1.0/) library is used to read the [SDFormat XML](http://sdformat.org/) file. Thus, the position of the cylinders is collected and a check is made to ensure that the robot never starts in a place already occupied by an obstacle.

### Environment exploration

Exploration is done using just a simple controller `turtlebot3_explorer` based on the [`turtlebot3_examples`](https://github.com/ROBOTIS-GIT/turtlebot3/tree/humble-devel) package. This ROS2 node subscribes to receive messages from the laser sensor and publishes velocity commands. If any obstacle is detected in front of the robot, it then rotates until it finds a free path again. Also has a service that allows to enable or disable this behavior.

<p align="center">
    <img src="etc/images/turtlebot3_explorer.png" alt="turtlebot3_explorer" width="400"/>
</p>

### Occupancy grid

The entire solution proposed here for counting obstacles is based on the use of an occupancy grid map. To generate this map, it was developed a ROS2 node `turtlebot3_occupancy_grid` that subscribes to receive messages from the laser sensor and updates the occupancy grid map for each message received. Initially, all points on the occupancy map have probability equal to 50%. As messages are received from the laser sensor, occupied points will have probability above 50% and free points on the map will have probability below 50%. This probabilistic occupancy grid is published at a fixed rate in the `/custom_map` topic. 


<p align="center">
    <img src="etc/images/turtlebot3_occupancy_grid.png" alt="turtlebot3_occupancy_grid" width="400"/>
</p>


The occupancy grid mapping algorithm uses the log-odds representation of occupancy:

$$l_{t,i} = log(\frac{p(m_i|z_{1:t},x_{1:t})}{1 - p(m_i|z_{1:t},x_{1:t})})$$

The probabilities are easily recovered from the log-odds ratio:

$$p(m_i|z_{1:t},x_{1:t}) = 1 - \frac{1}{1+ exp(l_{t,i})}$$

The algorithm occupancy grid mapping in below loops through all grid cells $i$, and updates those that were measured. The function `inverse_sensor_model` implements the inverse measurement model $p(m_i|z_{1:t},x_{1:t})$ in its log-odds form: if it measured any smaller than the maximum laser range, then mark the points on the map that are under the laser beam as free and the last one as occupied; if it measured some infinite value, truncated to max laser range and marks all as free grid cells. To accomplish this, an implementation of the [Bresenham line drawing algorithm](https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm) is used.

<p align="center">
    <img src="etc/images/occupancy_grid_algo.png" alt="occupancy_grid" width="400"/>
</p>

### Detect obstacles/objects


<p align="center">
    <img src="https://scipy-lectures.org/_images/sphx_glr_plot_labels_001.png" alt="connected_components" width="400"/>
</p>


<p align="center">
    <img src="etc/images/turtlebot3_object_detector.png" alt="turtlebot3_object_detector" width="400"/>
</p>

### Exploration end

<p align="center">
    <img src="etc/images/turtlebot3_mission_controller.png" alt="turtlebot3_mission_controller" width="400"/>
</p>



## Building

You can build all packages needed to run this assignment with docker:
```bash
docker build -t assignment-1:latest -f etc/docker/Dockerfile .
```

## Runnning

Not really safe, but it works.
```bash
sudo xhost +local:root
```

Then, create a network to run all containers in it:
```bash
docker network create my-net
```

Then, open a terminal in the container with support for the QT application:
```bash
docker run -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw --device /dev/dri/ --net=my-net assignment-1:latest /bin/bash
```

The first thing you'll need to do is run the gazebo inside the container:
```bash
gazebo
```

We haven't figured out why yet, but the first launch of the gazebo inside the container takes a long time. After the gazebo has opened the first time, you can close it and run our launch.
```bash
ros2 launch turtlebot3_mapper turtlebot3_mapper_launch.py
```

If you want to explore the environment, just open a new container in the same network and run the action client node.
```bash
docker run -it --net=my-net assignment-1:latest /bin/bash
ros2 run turtlebot3_mapper turtlebot3_mission_client -f 200
```

After the task is finished, you can view the results in the generated `results.txt` file.