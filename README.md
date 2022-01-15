# Vortex-ASV
Vortex ASV software. Purpose built for competing in ASV competitions. 


# Docker
Docker is a tool for creating a virtual environment with predetermined dependencies, much like a VM with fixed installation steps. In this case, we are using the the ROS noetic image as a base, and adding our own dependencies on top of it.
To use Docker, make sure you have downloaded the [Docker Engine](https://docs.docker.com/engine/install/ubuntu/) and [Docker compose](https://docs.docker.com/compose/install/).
To build the Docker image, navigate to this folder (Vortex-ASV) where the Dockerfile and docker-compose.yml files are located, and execute the command  

```
sudo docker-compose build
```

Note that you only need to build the image once unless changes have been made to the Dockerfile since the last time you built.
After building the image, you can run any service using

```
sudo docker-compose up -d <service_name>
```

Running without a service name runs every container listed in the docker-compose file. Available services are:
vortex - for running on hardware
vortex_asv_sim - for running on simulator
The -d flag (detach) runs the container in the background.

To open the terminal inside the container, run

```
sudo docker-compose exec <service_name> /bin/bash
```


In the case that you get the error message "bash: /home/vortex/asv_ws/devel/setup.bash: No such file or directory" when you enter the container terminal it means you have not built the ROS workspace. 

Build the ROS workspace: 

```
cd ~/asv_ws && catkin build
```

Now you can source by running

```
source ~/asv_ws/devel/setup.bash
```

or alternatively, it will source automatically the next time you run the container. To exit the container, run

```
exit
```  

When you are done working within the container, close it by running

```
sudo docker-compose down <service_name>
```

## Volumes
A volume is a folder which is linked between the container and the host the container the running on.  /home/vortex/asv_ws/ is a volume, meaning making any changes in the volume from the container also changes the asv_ws folder in the host. 

When new drivers are added, ensure that they are installed correctly through dependencies.repos, and make the driver directory a volume by adding it to the volumes in the vortex service in docker-compose.yml.

```
    - "../ouster_example:/home/vortex/asv_ws/src/ouster_example"
```
Note that the service vortex_sim does not need access to the drivers.