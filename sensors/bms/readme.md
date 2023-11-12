# Vortex-ASV BMS system

## Overview

This package contains a script for collecting BMS 
data, as well as a ROS node for publishing that 
data. The system is documented at [wiki page](https://vortex.a2hosted.com/index.php/Freya_BMS).

## ROS2

The package contains a node called freya_bms, which 
publishes BMS data to the topics 
* /internal/status/bms0
* /internal/status/bms1
* ... (bms topics are created for each battery pack that is connected)
* /diagnostics

The node publishes a message of type BatteryState to the bms topics, and a DiagnosticsArray message to the diagnostics topic.

The node can be run with:
```
ros2 launch bms_launch.py
```

or alternatively
```
ros2 run bms bms_publisher
```

## Contents

### BMS

This class contains the logic of Freya's BMS system. 

#### Methods
```
find_usb_ports(logger) -> list[str]:
    returns a list of USB ports to check for BMS data on.
get_bms_data() -> str | None:   
    returns pure BMS data string, or None if exception is thrown 
parse_bms_data(self, bms_data: str, logger) -> bool:
    parses bms_data and updates class members. Returns False if no data was sent in,
    returns true otherwise.
change_usb_port(self, usb_port: str) -> None:
    changes the usb port for the BMS.
```

#### Attributes
```
voltage (float): Voltage being delivered by the BMS
current (float): Current being delivered by the BMS
design_capacity (float): Capacity of the BMS
remaining_capacity (float): Remaining capacity of the BMS
percent_capacity (float): Remaining capacity as a float from 0-1
cycle_count (int): 
probes (int): 
strings (int): 
temps (Tuple[float, float, float]): Temperatures of the cell arrays
cells (Tuple[float, ...]): Voltages of individual cells (6 elements long)
balance (str): 
cell_total (float): Sum of cell voltages
cell_min (float): Smallest cell voltage
cell_max (float): Largest cell voltage
cell_diff (float): Difference between largest and smallest cell voltage
cell_avg (float): Average of cell voltages
device_name (str): Device name
manufacture_date (str): Date of manufacturing
version (str): Version
FET (str): 
```

### FreyaBMSNode

This class contains the node for publishing the BMS data to ROS.

#### Methods
```
publish_bms_data(self) -> None
    publises BMS data from BMS system to ros2 node.
create_key_value_pair(key: str, value) -> KeyValue:
    creates KeyValue object from supplied key and value
```

## Startup service

Note: The .service file may need to be edited in order to source the setup.bash file 
from the correct workspace.

This package also contains a .service file for running the node on startup. To enable this, just run (from this directory)
```
sudo cp startup_script/bms_startup.service /etc/systemd/system/bms_startup.service
```
then run the following commands
```
cd /etc/systemd/system
```
```
sudo systemctl daemon-reload && sudo systemctl enable bms_startup.service
```
To verify that the service is enabled, run
```
systemctl list-unit-files | grep enabled
```
The file will now run automatically on startup the next time the system starts.

If this fails, try checking if systemd is installed.

