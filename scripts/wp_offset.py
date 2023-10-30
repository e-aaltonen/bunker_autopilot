#!/usr/bin/python

"""
wp_backwards.py
Esa Aaltonen 2023

This node moves the entire mission (each WP) on the map according to distance (m) and
bearing (dgr) arguments set.

It reads the waypoint list from MavROS, moves each WP including home position (wps[0])
and sends the new list back.
 
Requirements:
- MavROS node (mavros_node) running

"""

import rospy
from mavros_msgs.msg import Waypoint, WaypointList
from mavros_msgs.srv import WaypointPush
import math
import time
import sys

class WPoffset():
    def __init__(self):
        rospy.init_node("wp_offset")
        self.sub_mission_wps = rospy.Subscriber("mavros/mission/waypoints", WaypointList, self.callback_mission_wps)
      
        rospy.loginfo("> WP offset node initiated")

        
        #Distance coefficients for distance approximations
        self.k_lat = 111194
        self.k_long = 50519

        # k_lat and k_long are expected to be already set by wp_amend_clear.py or by setting the params
        if rospy_has_param('autopilot/k_lat') & rospy_has_param('autopilot/k_long'):
            self.k_lat = rospy.get_param('autopilot/k_lat')
            self.k_long = rospy.get_param('autopilot/k_long')

        self.wp = Waypoint()
        self.wps = WaypointList()
        
        self.wps_received = False
        
        self.distance = 0.0
        self.safe_dist = 200.0

        # Check parameter safety_area_size: the maximum distance from starting point until rcout_to_cmd_vel.py sets velocity to 0
        if rospy.has_param('safety_area_size'):
            self.safe_dist = rospy.get_param('safety_area_size')

        if self.distance > (self.safe_dist * 2):
            rospy.loginfo("Offset value must not be outside safety area: %s".format(self.safe_dist))
            self.distance = 0.0

        if len(sys.argv)>2:
            try:
                self.distance = float(sys.argv[1])
            except rospy.ServiceException as e:
                rospy.loginfo("Distance (arg. 1) must be float: %s"%e)
        

        self.direction = 0.0
        if len(sys.argv)>2:
            try:
                self.direction = float(sys.argv[2])
                while self.direction < 0.0:
                    self.direction += 360.0
                while self.direction > 360.0:
                    self.direction -= 360.0

            except rospy.ServiceException as e:
                rospy.loginfo("Direction (arg. 2) must be float: %s"%e)

           
    # Add new WP as the last item on the list, using current GPS location
    def offsetWPlist(self): 
        rospy.loginfo("offset list")
        if (len(self.wps.waypoints) > 1):
            wp_list = WaypointList()
            wp_list.waypoints.append(self.wps.waypoints[0])
            for wpoff in self.wps.waypoints[1:]:
                wpoff = self.offsetWP(wpoff, self.distance, self.direction)
                wp_list.waypoints.append(wpoff)
            self.wps.waypoints = wp_list.waypoints

        #Send list to the FCU
        try:
            self.wpPush()
        except rospy.ServiceException as e:
            rospy.loginfo("Service call push waypoint failed: %s"%e)

    def offsetWP(self, wpset, dist, dxion):
        rospy.loginfo("offsetWP: lat {0} long{1}".format(wpset.x_lat, wpset.y_long))
        k_y = math.cos(math.radians(dxion))
        k_x = math.sin(math.radians(dxion))
        distY = dist * k_y / self.k_lat
        distX = dist * k_x / self.k_long
        wpset.x_lat += distY
        wpset.y_long += distX
        rospy.loginfo("now lat {0} long{1}".format(wpset.x_lat, wpset.y_long))
        return wpset
        
                
    # Send new WP list to the FCU
    def wpPush(self): 
        rospy.loginfo("Push")
        rospy.wait_for_service('mavros/mission/push')
        try:
            wpPushService = rospy.ServiceProxy('mavros/mission/push', WaypointPush)
            result = wpPushService(start_index = 0, waypoints = self.wps.waypoints)
            if result.success:
                rospy.loginfo("Sent new waypoint list.")
        except rospy.ServiceException as e:
            rospy.loginfo("Service call push waypoints failed: %s"%e)


    # Read WP list from the FCU
    def callback_mission_wps(self, data): 
        rospy.loginfo("CB mission")
        if self.wps.current_seq != data.current_seq:
            rospy.loginfo("Current mission waypoint sequence updated: {0}".format(data.current_seq))
        self.wps = data
        self.wps_received = True
        
            
            
    def run(self):
        rate = rospy.Rate(5)
        while not self.wps_received:
            rate.sleep()
        self.offsetWPlist()


if __name__ == "__main__":
    wp_proxy = WPoffset()
    wp_proxy.run()
    
