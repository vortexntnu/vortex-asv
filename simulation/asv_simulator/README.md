# ASV simulator
This package provides the implementation of the asv_simulator for the Vortex ASV.

## Usage

Launch controller and guidance `ros2 launch asv_setup hybridpath.launch.py`
Launch the asv_simulator `ros2 launch asv_simulator asv_simulator.launch.py`

## Configuration

You can configure the settings of the simulator and waypoints to be used by modifying the parameters in the `config` directory. 

## Foxglove 

To visualize the simulation in Foxglove, launch foxglove bridge `ros2 run foxglove_bridge foxglove_bridge`

