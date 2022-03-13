# Vortex-ASV
Vortex ASV software. Purpose built for competing in ASV competitions. 


# Docker, user guide
Docker is a tool for creating a virtual environment with predetermined dependencies, much like a VM with fixed installation steps. In this case, we are using the the ROS noetic image as a base, and adding our own dependencies on top of it.
To use Docker, make sure you have downloaded the [Docker Engine](https://docs.docker.com/engine/install/ubuntu/) and [Docker compose](https://docs.docker.com/compose/install/).

We use Docker to be able to run the code on *any* PC that has the Docker installed. This first part is the basics of how to run a docker container on your PC. For developing, skip to the next part

To run a service, navigate to Vortex-ASV where docker-compose.yml is located and run
```
sudo docker-compose up -d <service_name>
```
Where <service name> is a name from the list:
```
asv
lidar
simulator
```
The -d flag (detatch) runs the services in the background instead of taking up your terminal.
These are defined in [docker-compose.yml](docker-compose.yml). Alternatively you could run all services simultaneously. The services will run in different containers, but because of how they are defined they share their output between containers and the host. This means a ROS instance in any container can communicate with a ROS instance on the host or another container. Note that the first time you run a service it might take a few minutes.

Running the services in this way launches the programs associated with the services. For instance, running the "simulator" service launches the gazebo simulator with the GUI disabled. This makes it so any PC can run the simulator. Some PCs could not run the simulator, but with the GUI disabled, it should run fine. To get some visualization, use rviz:

```
rosrun rviz rviz -d sim_config.rviz
```

When you are finished using the container, run

```
sudo docker-compose down <service_name>
```
# Docker, developer guide

If you are altering the code, running the services listed above is cumbersome. The way you would run the code after making new changes new changes would be:
* Push your code
* The image is then rebuilt on a git cloud
* Run the container
Rather than going through this, we can used a feature in docker called volumes. A volume is a folder which is linked between the container and the host the container the running on.  /home/vortex/asv_ws/ is a volume, meaning making any changes in the volume from the container also changes the asv_ws folder in the host and vice versa.

Now the workflow is like this:
* Navigate to Vortex-ASV/
* Build the image locally using
```
sudo docker-compose build Dockerfile.dev
```
(You only need to do this once, unless changes have been made to (Dockerfile.dev)(Dockerfile.dev))
* Run the container using the developer services, ie a service in [docker-compose.yml](docker-compose.yml) ending in "-dev"
* Enter the terminal of the container you created:
```
sudo docker-compose exec <service name> /bin/bash
```
You can open as many terminals as you want. If you plan on opening many terminals, consider [creating an alias](https://www.cyberciti.biz/faq/create-permanent-bash-alias-linux-unix/)
* Run the code inside the container

The most common use of this will be running the simulator in one container and running the asv-dev in another for testing of new code. You might need to build the catkin workspace inside the container. This is no different from normal:
``` 
cd ~/asv_ws && catkin build 
source ~/asv_ws/devel/setup.bash
```
Now you can run and test any changes made, and make as many new ones as you'd like. to exit the container, run 
```
exit
```
When you are finished using the container, run

```
sudo docker-compose down <service_name>
```