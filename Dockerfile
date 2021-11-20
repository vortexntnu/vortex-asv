FROM ros:noetic

ARG distro=noetic
ENV DEBIAN_FRONTEND=noninteractive

# Create vortex user
RUN useradd -ms /bin/bash \
    --home /home/vortex  vortex
RUN echo "vortex:vortex" | chpasswd
RUN usermod -aG sudo vortex
RUN apt-get update && apt-get install -y python3-catkin-tools


# ROS package dependencies
RUN apt-get update && apt-get install -y \
    ros-$distro-roslint \
    ros-$distro-move-base-msgs \
    ros-$distro-tf \
    ros-$distro-tf2 \
    ros-$distro-eigen-conversions \
    ros-$distro-tf2-geometry-msgs \
    ros-$distro-pcl-ros \
    libeigen3-dev \
    libglfw3-dev \
    libglew-dev \
    libjsoncpp-dev \
    libtclap-dev


RUN echo "source /opt/ros/noetic/setup.bash" >> /home/vortex/.bashrc
RUN echo "source /home/vortex/asv_ws/devel/setup.bash" >> /home/vortex/.bashrc

RUN mkdir -p /home/vortex/asv_ws
RUN chown vortex /home/vortex/asv_ws

CMD ["/bin/bash"]