# Bunker_autopilot
by E. Aaltonen 2023/24

This is an autopilot solution for unmanned ground vehicles (UGV). An FCU (tested on Pixhawk 2 Cube and Pixhawk 6 C Mini) with ArduRover as well as MAVROS on an on-board computer are used to provide functionalities through ROS and a simple user interface based on a 10-channel RC transmitter. The package also includes a graphical interface, based on PyAutoGUI, using the RC transmitter as an input device for the on-screen application. The GUI was developed and tested in Gnome (Ubuntu).

The original purpose was to operate the FCU through the ROS system and MAVROS because the hardware setup on the target UGV (Agilex Bunker) would not allow the RC receiver and ESCs to be directly connected to the FCU. Instead, the on-board computer communicates with the control unit through the CAN bus to receive RC input data and to deliver maneuvering commands. Modified source code (for ROS packages by Agilex & Weston Robot) is required to make use of the RC input from the CAN bus; these are found in the *bunker_ros_RC* and *ugv_sdk_RC* repositories herewith. The system can be adapted for a "regular" setup, with the receiver, ESC and steering servo directly attached, by replacing necessary application-specific nodes with corresponding alternatives. Both versions share the same behaviour and switch functionalities, and the tasks are handled by this ROS application rather than the FCU/ArduRover.

The core part of the application is the *mission_server* node, running the "*mission_manip*" service. Calling this service with relevant attributes on the CLI or through the client nodes (using the RC switches or the GUI) enables the user to run the following mission-related tasks:<br>
0 SET_HOME (set current GPS location as Waypoint 0)<br>
1 ADD_WP (add current GPS location as the last WP, after the current WP or after a specified WP)<br>
2 REMOVE_WP (remove the last WP, the current WP or the specified WP)<br>
3 CLEAR_MISSION (remove all waypoints)<br>
4 BACKWARDS_MISSION (invert the waypoint list)<br>
5 OFFSET_MISSION (move each waypoint by a given distance (in metres) to a given direction (in degrees))<br>
6 SCALE_ROTATE_MISSION (resize the mission map by relocating each waypoint, based on a scale factor (1.0 = original size), and rotate the mission by a given amount (in degrees))<br>
7 MIRROR_MISSION (mirror the shape of the mission over a given mirror line running through the centre point, expressed in degrees)<br>

Tasks 1, 2 and 3 can be used with RC switches (activation by flipping SWA down and task selection based on VRA position: middle - ADD_WP, up - REMOVE_WP, down - CLEAR_MISSION). SWC is reserved for selecting ArduRover control mode and SWD for arming/disarming. Switching SWB down enables using the input (sticks for "mouse" & scrolling + SWA for "clicking") for the GUI. 

Developed with FlySky i6S (10 channels), using switches SWA (2-pos.), SWB (3-pos.), SWC (3-pos.) and SWD (2-pos.) plus potentiometer VAR_A. The GUI module also supports the rear button (KEY1) and both sticks, totalling 10 channels.

Mar 2024: Change switch position literals in accordance to new RCState messages (bunker_ros_RC & scout_ros_RC).

Mar 2025: New automatic slowdown/stop-and-wait functionality using a Velodyne lidar: When running a mission in AUTO mode and the lidar detects an obstacle in the front or lateral slow-down zone, the rcout_to_cmd_vel node will adjust speed based on proximity messages from the lidar_proximity node. When the obstacle is close enough, in the stop zone, the robot will stop (linear speed in cmd_vel messages set to zero) until the obstacle is removed. The lidar_proximity node can identify a solid obstacle as a slope (up to a defined angle), allowing the robot to continue movement. 

Usage: roslaunch bunker_autopilot scout_lidar_proximity.launch (for Scout)

Parameters:
- autopilot/robot_width: the robot's physical width (m)
- autopilot/floor_level: the level below the lidar's sight level under which points are ignored (m negative)
- autopilot/robot_front_edge: distance between the lidar and the robot's front edge (m positive)
- autopilot/robot_rear_edge: distance between the lidar and the robot's rear edge (m negative)
- autopilot/stop_range_lateral: the size of stop area on each side of the robot (m positive) - the robot stops if an obstacle is detected in this area
- autopilot/slowdown_range_lateral: the size of slow-down area on each side of the robot (m positive) - the robot slows down if an obstacle is detected in this area
- autopilot/stop_range_front: the size of stop area in front of the robot (m positive) - the robot stops if an obstacle is detected in this area
- autopilot/slowdown_range_front: the size of stop area on each side of the robot (m positive) - the robot slows down if an obstacle is detected in this area
- autopilot/max_slope: maximum angle (in degrees) qualified as a slope
- autopilot/rings: the number of lowermost laser rings used - the remaining rings are simply ignored

  
