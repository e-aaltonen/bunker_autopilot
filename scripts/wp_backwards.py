#!/usr/bin/python

"""
wp_backwards.py
Esa Aaltonen 2023

This node enables the rover to run the mission in reverse order from the last WP to first.

It reads the waypoint list from MavROS, inverts the list excluding home position (wps[0])
and sends the new list back.
 
Requirements:
- MavROS node (mavros_node) running

"""

import rospy
from mavros_msgs.msg import Waypoint, WaypointList
from mavros_msgs.srv import WaypointPush, WaypointPull, WaypointClear

import time

class WPbackw():
    def __init__(self):
        rospy.init_node("wp_backwards")
        self.sub_mission_wps = rospy.Subscriber("mavros/mission/waypoints", WaypointList, self.callback_mission_wps)

        self.wps = WaypointList()
        
        self.wps_received = False
        

           
    # Invert WP list after home position (excluding [0])
    def invertWPlist(self): 
        rospy.loginfo("Number of waypoints: {0}".format(len(self.wps.waypoints)))
        rospy.loginfo("Last WP lat: {0}".format(self.wps.waypoints[-1].x_lat))
        if(len(self.wps.waypoints) > 2):
            self.wps.waypoints[1:] = self.wps.waypoints[len(self.wps.waypoints):0:-1]
        rospy.loginfo("Last WP lat: {0}".format(self.wps.waypoints[-1].x_lat))
        
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
                rospy.loginfo("Sent new waypoint list.")
        except rospy.ServiceException as e:
            rospy.loginfo("Service call push waypoints failed: %s"%e)


    # Read WP list from the FCU
    def callback_mission_wps(self, data): 
        if self.wps.current_seq != data.current_seq:
            rospy.loginfo("Current mission waypoint sequence updated: {0}".format(data.current_seq))
        self.wps = data
        self.wps_received = True

        
    def run(self):
        rate = rospy.Rate(5)
        while not self.wps_received:
            rate.sleep()
        self.invertWPlist()


if __name__ == "__main__":
    wp_proxy = WPbackw()
    wp_proxy.run()

