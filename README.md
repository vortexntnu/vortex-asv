# Vortex-ASV
Vortex ASV software. Purpose built for competing in ASV competitions. 


# Docker
Docker is a tool for creating a container, in this case running the ROS noetic image,
To use Docker, make sure you have downloaded [Docker Engine](https://docs.docker.com/engine/install/ubuntu/) and [Docker compose](https://docs.docker.com/compose/install/).
To build the Docker image, navigate to this folder (Vortex-ASV) int the terminal and execute the command 
'''
$ sudo docker-compose build
'''
Note that you only need to build the image once unless changes have been made to the Dockerfile since the last time you built.
After building the image, you can run any service using
'''
$ sudo docker-compose up -d <service_name>
'''
Running without a service name runs every container in the docker-compose file.
The -d flag (detatch) runs the container in the background.
When you are done working within the container, close it by running
'''
$ sudo docker-compose down <service_name>
'''
To open the terminal inside the container, run
'''
$ sudo docker-compose exec vortex /bin/bash
'''
If you get the error message "bash: /home/vortex/asv_ws/devel/setup.bash: No such file or directory" when you enter the container terminal it means you have not built the workspace. Navigate to /home/vortex/asw_ws and run
'''
$ catkin build
'''
Now you can source by running
'''
$ source source /home/vortex/asv_ws/devel/setup.bash
'''
or alternatively, it will source automatically the next time you run the container. To exit the terminal, run
'''
$ exit
'''

## volumes
A volume is a folder wich is linked between the container and the host the container the running on. The folder asw_ws in this repository is a volume, meaning any changes made inside the folder is automatically synced. /home/vortex/asv_ws/ is a volume, meaning making any changes in the volume from the container also changes the asv_ws folder in the host. 

Right now, the container is pretty empty, but it will be updated with the dependencies needed to run the ASV as the project continues.