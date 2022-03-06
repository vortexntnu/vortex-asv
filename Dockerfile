FROM ros:noetic

ARG distro=noetic
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"] 

# ROS package dependencies
RUN apt update && \
    apt install -y \
    ros-$distro-roslint \
    ros-$distro-move-base-msgs \
    ros-$distro-tf \
    ros-$distro-tf2 \
    ros-$distro-eigen-conversions \
    libeigen3-dev \
    ros-$distro-joy \
    ros-$distro-tf2-geometry-msgs \
    ros-$distro-geographic-msgs \
    ros-$distro-pcl-ros \
    ros-$distro-rviz \
    ros-$distro-rtabmap \
    ros-$distro-rtabmap-ros \
    ros-noetic-imu-tools \
    libeigen3-dev \
    libglfw3-dev \
    libglew-dev \
    libjsoncpp-dev \
    libtclap-dev \
    libgeographic-dev \
    python3-catkin-tools \
    python3-vcstool \
    net-tools \     
    tcpdump 

COPY . /vortex_ws/src
RUN source /opt/ros/noetic/setup.bash && cd /vortex_ws && catkin build

COPY ./entrypoint.sh /
CMD ["/entrypoint.sh"]
