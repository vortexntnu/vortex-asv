## ASV setup

This package contains global configs and the main launchfiles. The reason for putting these in a separate package is to be able to reach the aforementioned files from anywhere in a simple manner.

### Config
The config folder contains physical parameters related to the AUV and the environment

There are currently no physical parameters

### Launch
This package contains a launchfile for each specific ASV. Additionally the pc.launch file is to
be used on the topside computer that the joystick is connected to, for ROV operations.

naglfar.launch is currently empty, except for input arguments

For the ASV launchfiles, the following parameters can be used:

| Parameter | Options         | Default   |
| ----------|-----------------|-----------|
| type      | real, simulator | simulator |


