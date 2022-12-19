ARG ROS_DISTRO=humble
FROM osrf/ros:${ROS_DISTRO}-desktop-full

SHELL [ "/bin/bash" , "-c" ]
ENV DEBIAN_FRONTEND=noninteractive
ENV NVIDIA_VISIBLE_DEVICES=${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES=${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

ARG ROS_WS=/opt/ros2_ws
ENV ROS_WS=${ROS_WS}

RUN apt-get update && \
    apt-get install --no-install-recommends -y \
    python3-pip \
    ros-${ROS_DISTRO}-vision-msgs \
    ros-${ROS_DISTRO}-turtlebot3* && \
    apt-get autoremove -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --upgrade \
        pip \
        numpy \
        opencv-python \
        setuptools==58.2.0 \
        python-sdformat==0.4.0

COPY src/worlds_gazebo/worlds/autonomous_robots_world_1.world /opt/ros/humble/share/turtlebot3_gazebo/worlds/
COPY src/worlds_gazebo/worlds/autonomous_robots_world_2.world /opt/ros/humble/share/turtlebot3_gazebo/worlds/

COPY src/worlds_gazebo/models/ /opt/ros/humble/share/turtlebot3_gazebo/models/
COPY src/worlds_gazebo/launch/turtlebot3_world.launch.py /opt/ros/humble/share/turtlebot3_gazebo/launch/

ENV TURTLEBOT3_MODEL=waffle
ENV GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models

WORKDIR ${ROS_WS}
COPY src/ ${ROS_WS}/src/

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --packages-select turtlebot3_mapper turtlebot3_interfaces

COPY /etc/scripts/ros2_ws_entrypoint.sh /ros2_ws_entrypoint.sh
ENTRYPOINT [ "/ros2_ws_entrypoint.sh" ]