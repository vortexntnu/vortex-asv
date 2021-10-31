FROM ros:noetic

#Create vortex user with sudo permissions
RUN adduser --quiet --disabled-password --shell /bin/bash \
    --home /home/vortex --gecos "Vortex user for running ROS" vortex
RUN echo "vortex:vortex" | chpasswd
RUN usermod -aG sudo vortex

RUN apt-get update
RUN apt-get update --fix-missing


#Add dependencies like this:
# #SLAM dependencies
# RUN apt-get install -y --allow-unauthenticated \
#     SLAM-dependency-1 \
#     SLAM-dependency-2 \
#     SLAM-dependency-3

CMD ["/bin/bash"]