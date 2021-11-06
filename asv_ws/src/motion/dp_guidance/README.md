## ASV note
This should in principle not exist if we make the interface between FSM/GNC better than it was for the AUV.

## DP guidance node
This node simply passes setpoints made by the mission planner/state machine to the dp controller, and
exists by principle of separating state machine from controller.
