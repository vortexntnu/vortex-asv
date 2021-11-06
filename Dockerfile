FROM ros:noetic

# Create vortex user
RUN useradd -ms /bin/bash \
    --home /home/vortex  vortex
RUN echo "vortex:vortex" | chpasswd
RUN usermod -aG sudo vortex

RUN apt-get update

RUN apt-get install -y python3-catkin-tools

RUN echo "source /opt/ros/noetic/setup.bash" >> /home/vortex/.bashrc
RUN echo "source /home/vortex/asv_ws/devel/setup.bash" >> /home/vortex/.bashrc
CMD ["/bin/bash"]