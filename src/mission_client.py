#!/usr/bin/python
"""
mission_client.py
Esa Aaltonen 2023

Read remote control switch A and left knob VRA to insert/remove waypoints

This node subscribes to /switch/a and /switch/var_a published by rc_state_sub-pub.py
(uint8 for /switch/a, int8 for /switch/var_a) and calls service /bunker_autopilot/mission_manip
to manipulate the waypoint list.

***
Functions triggered by switch A down (at state change):
- if VRA middle: append new waypoint at the end of the list, using current position
- if VRA down (< -25): clear waypoint list
- if VRA up (> 25): remove waypoint at current_seq (waypoint currently navigated to if in AUTO mode, or last waypoint if mission finished)
***

Requirements:
- ugv_sdk (Weston Robot, v1.x) and bunker_base (Agilex Robotics) packages 
    (modified versions; see https://github.com/e-aaltonen/bunker_ros_RC and https://github.com/e-aaltonen/ugv_sdk_RC/tree/v1.x)
- CAN up (sudo ip link set can0 up type can bitrate 500000)
- bunker_bringup running (bunker_minimal.launch)
- rc_state_sub-pub.py running
- MavROS node (mavros_node) running

"""

import rospy
from std_msgs.msg import UInt8, Int8, Float32
from bunker_autopilot.srv import MissionManip
import time

# int literals - switch positions
SW_UP = 2
SW_MIDDLE = 1
SW_DOWN = 3

# MissionManip.srv literals for task field (int8)
SET_HOME = 0
ADD_WP = 1
REMOVE_WP = 2
CLEAR_MISSION = 3
BACKWARDS_MISSION = 4
OFFSET_MISSION = 5
SCALE_MISSION = 6
ROTATE_MISSION = 7

class WPmanip():
    def __init__(self):
        rospy.init_node("wp_manip")
        self.sub_swa = rospy.Subscriber("switch/a", UInt8, self.callback_update_swa)
        self.sub_swb = rospy.Subscriber("switch/b", UInt8, self.callback_update_swb)
        self.sub_var_a = rospy.Subscriber("switch/var_a", Int8, self.callback_update_var_a)

        rospy.loginfo("> wp_manip node initiated")

        self.swa = 0
        self.swb = 0
        self.var_a = 0

            
    # Check whether Switch A position has changed
    def callback_update_swa(self, msg):
        if msg.data != self.swa:
            self.swa = msg.data

            if msg.data == SW_DOWN and self.swb != SW_DOWN: # 3 = switch down to activate function, unless SWB is down
                # left knob turned left (down): clear mission
                if self.var_a < -25:
                    try:
                        rospy.wait_for_service('mission_manip')
                        wpService = rospy.ServiceProxy('mission_manip', MissionManip)
                        serviceSuccess = wpService(task = CLEAR_MISSION)
                    except rospy.ServiceException as e:
                        rospy.loginfo("Service call CLEAR_MISSION failed: %s"%e)
                
                # left knob turned right (up): remove current WP
                if self.var_a > 25: 
                    try:
                        rospy.wait_for_service('mission_manip')
                        wpService = rospy.ServiceProxy('mission_manip', MissionManip)
                        serviceSuccess = wpService(task = REMOVE_WP, use_last = False, use_current = True, seq = 0)
                    except rospy.ServiceException as e:
                        rospy.loginfo("Service call REMOVE_WP failed: %s"%e)
                
                # left knob in the middle: push WP
                if (self.var_a > -26) & (self.var_a < 26): 
                    # check whether GPS position was received
                    try:
                        rospy.wait_for_service('mission_manip')
                        wpService = rospy.ServiceProxy('mission_manip', MissionManip)
                        serviceSuccess = wpService(task = ADD_WP, use_last = False, use_current = True, seq = 0)
                    except rospy.ServiceException as e:
                        rospy.loginfo("Service call ADD_WP failed: %s"%e)

    def callback_update_swb(self, msg):
        if msg.data != self.swb:
            self.swb = msg.data
                                        
    # Read left knob value
    def callback_update_var_a(self, msg): 
        self.var_a = msg.data

        
    def run(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == "__main__":
    wp_proxy = WPmanip()
    wp_proxy.run()

