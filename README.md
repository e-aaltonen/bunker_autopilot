# Bunker_autopilot
by E. Aaltonen 2023/24

This project uses an FCU (tested with Pixhawk 2 Cube and Pixhawk 6 C Mini)  with ArduRover, and MAVROS on an on-board computer, to provide autopilot functionalities through ROS and a simple user interface based on a 10-channel RC transmitter. The package also includes a graphical interface running PyAutoGUI to use the RC transmitter as an input device for the on-screen application.

The original purpose was to operate the FCU through the ROS system and MAVROS because the hardware setup on the target UGV (Agilex Bunker) would not allow the RC receiver and ESCs to be directly connected to the FCU. Instead, the on-board computer communicates with the control unit through the CAN bus to receive RC input data and to deliver maneuvering commands. Modified source code is required to make use of the RC input from the CAN bus (found in the bunker\_ros\_RC and ugv\_sdk\_RC repositories). The system can be adapted for a "regular" setup, with the receiver, ESC and steering servo directly attached, by replacing necessary application-specific nodes with corresponding alternatives. Both alternatives share the same behaviour and switch functions, and the functions are handled by this ROS application rather than the FCU/ArduRover.

The core part of the application is the mission_server node, running the "mission_manip" service. Calling this service with relevant attributes on the CLI or through the client nodes (used with the RC switches or the GUI) enables the user to run the following mission-related tasks:<br>
0 SET_HOME (set current GPS location as Waypoint 0)<br>
1 ADD_WP (add current GPS location as the last WP, after the current WP or after a specified WP)<br>
2 REMOVE_WP (remove the last WP, the current WP or the specified WP)<br>
3 CLEAR_MISSION (remove all waypoints)<br>
4 BACKWARDS_MISSION (invert the waypoint list)<br>
5 OFFSET_MISSION (move each waypoint a given distance (in metres) to a given direction (in degrees))<br>
6 SCALE_ROTATE_MISSION (resize the mission map by relocating each waypoint, based on a scale factor (1.0 = original size), and rotate the mission by a given amoung in degrees)<br>
7 MIRROR_MISSION (mirror the shape of the mission over a given mirror line running through the centre point, expressed in degrees)<br>

Tasks 1, 2 and 3 can be used with RC switches.

