# Vortex-ASV
Vortex ASV software. Purpose built for competing in ASV competitions. 


# Docker
Docker is a tool for creating a container, in this case running the ROS neotic image,
To use ducker, make sure you have downloaded [Docker Engine](https://docs.docker.com/engine/install/ubuntu/) and [Docker compose](https://docs.docker.com/compose/install/).
To build the Docker image, navigate to this folder (Vortex-ASV) int the terminal and execute the command 
'''
sudo docker-compose build
'''
After building the image, you can run any service using
'''
sudo docker-compose up -d <service_name>
'''

Right now, the container is pretty empty, but it will be updated with the dependencies needed to run the ASV as the project continues