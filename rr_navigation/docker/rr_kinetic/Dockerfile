# This is an auto generated Dockerfile for ros:desktop-full
# generated from docker_images/create_ros_image.Dockerfile.em
FROM osrf/ros:kinetic-desktop-xenial

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-kinetic-desktop-full=1.3.2-0* 

# install basic tools
RUN apt-get install -y iputils-ping iproute2 net-tools screen byobu vim

RUN touch /root/.screenrc && echo "shell /bin/bash" > /root/.screenrc

#install ros navigation package
RUN echo "Installing extra ROS packages"
RUN apt-get install -y ros-kinetic-laser-scan-matcher ros-kinetic-gmapping \
    python-rosdep python-rosinstall python-rosinstall-generator python-wstool\
    build-essential python-rosdep ros-kinetic-dwa-local-planner ros-kinetic-frontier-exploration

RUN echo "Setting up catkin workspace"
RUN mkdir -p /root/catkin_ws/src 
RUN chmod +x "/opt/ros/$ROS_DISTRO/setup.bash"
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && \
                  cd /root/catkin_ws && catkin_make"

RUN mkdir -p /data/bin
COPY data/bin /data/bin
RUN chmod +x /data/bin/*

RUN echo "source /data/bin/env_setup.sh" >> /root/.bashrc 

ENTRYPOINT ["/data/bin/entrypoint.sh"]