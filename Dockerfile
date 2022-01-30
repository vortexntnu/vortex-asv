FROM ros:noetic

ARG distro=noetic

# Create vortex user
RUN useradd -ms /bin/bash \
    --home /home/vortex  vortex
RUN echo "vortex:vortex" | chpasswd
RUN usermod -aG sudo vortex

RUN apt-get update

RUN apt-get install -y python3-catkin-tools

# ROS package dependencies
RUN apt-get install -y \
    ros-$distro-roslint \
    ros-$distro-move-base-msgs \
    ros-$distro-tf \
    ros-$distro-tf2 \
    ros-$distro-eigen-conversions \
    libeigen3-dev \
    ros-$distro-joy

RUN echo "source /opt/ros/noetic/setup.bash" >> /home/vortex/.bashrc
RUN echo "source /home/vortex/asv_ws/devel/setup.bash" >> /home/vortex/.bashrc

RUN mkdir -p /home/vortex/asv_ws
RUN chown vortex /home/vortex/asv_ws

CMD ["/bin/bash"]