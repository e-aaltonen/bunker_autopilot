#!/usr/bin/python
"""
wp_amend_clear.py
Esa Aaltonen 2023

[Oct 2023: untested changes to calculateCoefficients(), distanceNextWP()]

Read remote control switch A and left knob VRA to insert/remove waypoints

This node subscribes to /switch/a and /switch/var_a published by rc_state_sub-pub.py
(uint8 for /switch/a, int8 for /switch/var_a) to manipulate waypoint list as well as to
/mavros/mission/waypoints to receive waypoint list from the FCU and to
/mavros/global_position/global to receive current GPS position.

After amending local waypoint list, the list is updated to the FCU by calling service
/mavros/mission/push. The waypoint list is cleared (waypoint list emptied except home position) by
calling /mavros/mission/clear. Calling service /mavros/mission/pull is used to verify the
current number of waypoints on the FCU. This is also necessary to update the list and to verify that
the list published in topic /mavros/mission/waypoints is actually correct.

***
Functions triggered by switch A down (at state change):
- if VRA middle: append new waypoint at the end of the list, using current position
- if VRA down (< -25): clear waypoint list
- if VRA up (> 25): remove waypoint at current_seq (waypoint currently navigated to if in AUTO mode, or last waypoint if mission finished)
***

The node also publishes relevant navigation information in topic /wp_info:
uint8 total_wps             - total number of waypoints on the list
uint8 next_wp               - current navigation index
float32 distance_to_next_wp - distance in metres
float32 target_bearing      - in degrees, 0 = north
Distances are calculated using a 2D approximation for short distances

Requirements:
- ugv_sdk (Weston Robot, v1.x) and bunker_base (Agilex Robotics) packages
- CAN up (sudo ip link set can0 up type can bitrate 500000)
- bunker_bringup running (bunker_minimal.launch) (modified code)
- rc_state_sub-pub.py running
- MavROS node (mavros_node) running

"""

import rospy
import math
from std_msgs.msg import UInt8, Int8
from mavros_msgs.msg import Waypoint, WaypointList
from mavros_msgs.srv import WaypointPush, WaypointPull, WaypointClear
from sensor_msgs.msg import NavSatFix
from autopilot_test.msg import WPInfo
import subprocess

import time

class WPmanip():
    def __init__(self):
        rospy.init_node("wp_amend_clear")
        self.sub_swa = rospy.Subscriber("switch/a", UInt8, self.callback_update_swa)
        self.sub_var_a = rospy.Subscriber("switch/var_a", Int8, self.callback_update_var_a)
        self.sub_mission_wps = rospy.Subscriber("mavros/mission/waypoints", WaypointList, self.callback_mission_wps)
        self.sub_global_pos = rospy.Subscriber("mavros/global_position/global", NavSatFix, self.callback_global_position)        

        self.pub_wpinfo = rospy.Publisher("wp_info", WPInfo, queue_size=1)
        self.wpinfo_msg = WPInfo()
      
        rospy.loginfo("> WP amend / clear mission node initiated")

        self.swa = 0
        self.var_a = 0
        
        #Distance coefficients for distance approximations
        self.k_lat = 111194
        self.k_long = 50519
        self.k_set = False

        if rospy_has_param('autopilot/k_lat') & rospy_has_param('autopilot/k_long'):
            self.k_lat = rospy.get_param('autopilot/k_lat')
            self.k_long = rospy.get_param('autopilot/k_long')
            self.k_set = True

        self.global_position = NavSatFix()        
        self.got_gp = False
        self.wp = Waypoint()
        self.wps = WaypointList()

           
    # Add new WP as the last item on the list, using current GPS location
    def amendLocalWP(self): 
        #Update local list
        self.wp.frame = 3 #  FRAME_GLOBAL_REL_ALT = 3
        self.wp.command = 16 # NAV_WAYPOINT = 16
        self.wp.is_current= False
        self.wp.autocontinue = True
        self.wp.param1 = 0.0 
        self.wp.param2 = 0.0
        self.wp.param3 = 0.0
        self.wp.param4 = 0.0
        self.wp.x_lat = self.global_position.latitude
        self.wp.y_long = self.global_position.longitude
        self.wp.z_alt = self.global_position.altitude
        self.wps.waypoints.append(self.wp)
        
        #Send list to the FCU
        try:
            self.wpPush()
        except rospy.ServiceException as e:
            rospy.loginfo("Service call push waypoint failed: %s"%e)
        
    # Remove item at current_seq from the WP list
    def removeCurrentWP(self): 
        #Update local list
        if len(self.wps.waypoints) > 1:
            if self.wps.current_seq < len(self.wps.waypoints):
                # remove WP at current_seq
                self.wps.waypoints.pop(self.wps.current_seq)    
            else:
                # in case current_seq points outside the list, remove last WP
                self.wps.waypoints.pop()                        

            #Send list to the FCU
            try:
                self.wpPush()
            except rospy.ServiceException as e:
                rospy.loginfo("Service call push waypoint failed: %s"%e)     
        
    # Send new WP list to the FCU
    def wpPush(self): 
        rospy.wait_for_service('mavros/mission/push')
        try:
            wpPushService = rospy.ServiceProxy('mavros/mission/push', WaypointPush)
            result = wpPushService(start_index = 0, waypoints = self.wps.waypoints)
            if result.success:
                rospy.loginfo("Sent new waypoint list. Number of waypoints: {0}".format(self.wpPull()))
        except rospy.ServiceException as e:
            rospy.loginfo("Service call push waypoints failed: %s"%e)

    # Request current number of WPs
    def wpPull(self): 
        rospy.wait_for_service('mavros/mission/pull')
        try:
            wpPullService = rospy.ServiceProxy('mavros/mission/pull', WaypointPull)
            
            wp_num = 0
            res = wpPullService()
            if res.success:
                wp_num = res.wp_received
            #return wpPullService().wp_received
            return wp_num

        except rospy.ServiceException as e:
            rospy.loginfo("Service call pull waypoint failed: %s"%e)

    # Remove all waypoints on FCU WP list
    def clearMission(self): 
        # Instead of calling mavros/mission/clear service, which also clears home wp,
        # just clear local list and add current coordinates as new home
        del self.wps.waypoints[:]

        try:
            if (self.global_position.latitude > 0.0): 
                self.wp.frame = 0 #  FRAME_GLOBAL_REL_ALT = 3
                self.wp.command = 16 # NAV_WAYPOINT = 16
                self.wp.is_current = True
                self.wp.autocontinue = True
                self.wp.param1 = 0.0 
                self.wp.param2 = 0.0
                self.wp.param3 = 0.0
                self.wp.param4 = 0.0
                self.wp.x_lat = self.global_position.latitude
                self.wp.y_long = self.global_position.longitude
                self.wp.z_alt = self.global_position.altitude
                self.wps.waypoints.append(self.wp)
                rospy.loginfo("Waypoint list cleared. Current position set as new home location")
               
                self.wpPush()
                
        except rospy.ServiceException as e:
            rospy.loginfo("Set home failed: %s"%e)
            
    # Check whether Switch A position has changed
    def callback_update_swa(self, msg):
        if msg.data != self.swa:
            rospy.loginfo(self.var_a)
            self.swa = msg.data

            if msg.data == 3: # 3 = switch down to activate function
                # left knob turned left (down): clear mission
                if self.var_a < -25: 
                    self.clearMission()
                
                # left knob turned right (up): remove current WP
                if self.var_a > 25: 
                    self.removeCurrentWP()
                
                # left knob in the middle: push WP
                if (self.var_a > -26) & (self.var_a < 26): 
                    # check whether GPS position was received
                    if (self.global_position.latitude > 0.0): 
                        self.amendLocalWP()
                
    # Read left knob value
    def callback_update_var_a(self, msg): 
        self.var_a = msg.data

    # Read WP list from the FCU
    def callback_mission_wps(self, data): 
        if self.wps.current_seq != data.current_seq:
            rospy.loginfo("Current mission waypoint sequence updated: {0}".format(data.current_seq))
        self.wps = data
        self.wpinfo_msg.total_wps = len(self.wps.waypoints)

        self.wpinfo_msg.next_wp = self.wps.current_seq

    # Calculate good-enough approximates
    def calculateCoefficients(self):
        # adjusted equatorial diameter to compensate differences between the geoid and a sphere, for  lat ~63
        diagEq = 12793172 
        self.k_lat = math.pi * diagEq / 360
        try:
            self.k_long = math.pi * math.degrees(math.cos(self.global_position.latitude)) * diagEq / 360
        except rospy.ServiceException as e: 
            rospy.loginfo("Calculating k_long: %s"%e)

        self.k_set = True
        rospy.set_param('autopilot/k_lat', self.k_lat)
        rospy.set_param('autopilot/k_long', self.k_long)
                    
    # Calculate distance & bearing to next WP
    def distanceNextWP(self):
        if !self.k_set && self.got_gp:
            self.calculateCoefficients()

        nextwp = 1
        if (self.wps.current_seq < len(self.wps.waypoints)) & (self.wps.current_seq > 0):
            nextwp = self.wps.current_seq
        # here x = longitude, y = latitude
        diffy = (self.wps.waypoints[nextwp].x_lat - self.global_position.latitude)
        diffx = (self.wps.waypoints[nextwp].y_long - self.global_position.longitude)
        
        disty = abs(diffx * self.k_lat)
        distx = abs(diffy * self.k_long)
        dist = math.sqrt(disty**2 + distx**2)
        
        self.wpinfo_msg.distance_to_next_wp = dist
        
        t_bearing = 0
        try:
            if dist > 0:
                rospy.loginfo("Disty: {0}, dist: {1}".format(disty,dist))
                theta = math.acos(disty/dist)
                t_bearing = math.degrees(theta)

                if distx < 0:
                    t_bearing = 360 - t_bearing
            dist = round(dist,2)
            #rospy.loginfo("dist: {0}, bearing: {1}".format(dist,t_bearing))
            self.wpinfo_msg.target_bearing = t_bearing
        except rospy.ServiceException as e:
            rospy.loginfo("Bearing calculation failed: %s"%e)
    
    # Read GPS position data
    def callback_global_position(self, data): 
        self.global_position = data
        self.got_gp = True
        
    def run(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            if len(self.wps.waypoints) > 1:
                self.distanceNextWP()
                self.pub_wpinfo.publish(self.wpinfo_msg)
            rate.sleep()


if __name__ == "__main__":
    wp_proxy = WPmanip()
    wp_proxy.run()

