FROM ros:noetic

#Create vortex user with sudo permissions
RUN adduser --quiet --disabled-password --shell /bin/bash \
    --home /home/vortex --gecos "Vortex user for running ROS" vortex
RUN echo "vortex:vortex" | chpasswd
RUN usermod -aG sudo vortex

CMD ["/bin/bash"]