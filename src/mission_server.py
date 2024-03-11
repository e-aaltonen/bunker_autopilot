#!/usr/bin/env python
"""
mission_server.py
E. Aaltonen 5 Mar. 2024

This service subscribes to /mavros/mission/waypoints to receive waypoint list from the FCU and to
/mavros/global_position/global to receive current GPS position.

After amending local waypoint list, the list is updated to the FCU by calling service
/mavros/mission/push. The waypoint list is cleared (waypoint list emptied except home position) by
calling /mavros/mission/clear. Calling service /mavros/mission/pull is used to verify the
current number of waypoints on the FCU. This is also necessary to update the list and to verify that
the list published in topic /mavros/mission/waypoints is actually correct.

***
Function set by the task field:

SET_HOME = 0            ()
ADD_WP = 1              (bool use_last, bool use_current, int8 seq)
REMOVE_WP = 2           (bool use_last, bool use_current, int8 seq)
CLEAR_MISSION = 3       ()
BACKWARDS_MISSION = 4   ()
OFFSET_MISSION = 5      (bool all_wps, int8 seq, float32 distance, float32 direction_angle)
SCALE_MISSION = 6       (float32 scale_factor)
ROTATE_MISSION = 7      (float32 direction_angle)
***

The node also publishes relevant navigation information in topic /wp_info:
uint8 total_wps             - total number of waypoints on the list
uint8 next_wp               - current navigation index
float32 distance_to_next_wp - distance in metres
float32 target_bearing      - in degrees, 0 = north
Distances are calculated using a 2D approximation for short distances

"""


import rospy
from std_msgs.msg import Int8, Float32
from bunker_autopilot.srv import MissionManip, MissionManipResponse
from mavros_msgs.msg import Waypoint, WaypointList
from mavros_msgs.srv import WaypointPush, WaypointPull, WaypointClear
from sensor_msgs.msg import NavSatFix
from bunker_autopilot.msg import WPInfo
import math
import time

class distVector():
    def __init__(self, dist=0.0, dxion=0.0):
        self.dist = dist
        self.dxion = dxion

class WPmanip():
    def __init__(self):
        rospy.init_node('mission_server')
        s = rospy.Service('mission_manip', MissionManip, self.handle_task)
        rospy.loginfo("Service mission_manip initiated")

        self.sub_mission_wps = rospy.Subscriber("mavros/mission/waypoints", WaypointList, self.callback_mission_wps)
        self.sub_global_pos = rospy.Subscriber("mavros/global_position/global", NavSatFix, self.callback_global_position)        
        self.pub_wpinfo = rospy.Publisher("wp_info", WPInfo, queue_size=1)
        self.wpinfo_msg = WPInfo()

        self.global_position = NavSatFix()        
        self.got_gp = False
        self.wp = Waypoint()
        self.wps = WaypointList()
        
        #Distance coefficients for distance approximations
        self.k_lat = 111194
        self.k_long = 50519
        self.k_set = False

        if rospy.has_param('autopilot/k_lat') and rospy.has_param('autopilot/k_long'):
            self.k_lat = rospy.get_param('autopilot/k_lat')
            self.k_long = rospy.get_param('autopilot/k_long')
            self.k_set = True

        
    # Send new WP list to the FCU
    def wpPush(self): 
        rospy.wait_for_service('mavros/mission/push')
        try:
            wpPushService = rospy.ServiceProxy('mavros/mission/push', WaypointPush)
            result = wpPushService(start_index = 0, waypoints = self.wps.waypoints)
            if result.success:
                rospy.loginfo("Sent new waypoint list. Number of waypoints: {0}".format(self.wpPull()))
                return True
        except rospy.ServiceException as e:
            rospy.loginfo("Service call push waypoints failed: %s"%e)
            return False

    # Request current number of WPs
    def wpPull(self):
        rospy.wait_for_service('mavros/mission/pull')
        try:
            wpPullService = rospy.ServiceProxy('mavros/mission/pull', WaypointPull)
            
            wp_num = 0
            res = wpPullService()
            if res.success:
                wp_num = res.wp_received
            return wp_num
        
        except rospy.ServiceException as e:
            rospy.loginfo("Service call Pull waypoint failed: %s"%e)
            

    # *** SET_HOME ***
    # Set Home at current location
    def setCurrentHome(self): 
        succ = False

        if len(self.wps.waypoints) > 0:
            self.wps.waypoints[0].x_lat = self.global_position.latitude
            self.wps.waypoints[0].y_long = self.global_position.longitude
            self.wps.waypoints[0].z_alt = self.global_position.altitude
           
            succ = self.wpPush()
                
        return succ
        
    # *** ADD_WP ***
    # Add new WP as the last item on the list, using current GPS location
    def amendLocalWP(self, use_last, use_current, addSeq): 
        succ = False

        #Update local list
        if (self.global_position.latitude > 0.0): # Make sure global position data has been received
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

            if use_last:
                # add current position as last WP
                self.wps.waypoints.append(self.wp)
                succ = self.wpPush()
                return succ
            if use_current:
                # add current position at current seq
                self.wps.waypoints.insert(self.wps.current_seq, self.wp)
                succ = self.wpPush()
                return succ
            if addSeq < len(self.wps.waypoints):
                # add current position at indicated position (addSeq)
                self.wps.waypoints.insert(addSeq, self.wp)
                succ = self.wpPush()

        return succ        

    # *** REMOVE_WP ***
    # Remove item at current_seq from the WP list
    def removeCurrentWP(self, use_last, use_current, remSeq): 
        succ = False

        #Update local list
        if len(self.wps.waypoints) > 1:
            if use_last:
                # remove last WP
                self.wps.waypoints.pop()
                succ = self.wpPush()
                return succ
            if use_current:
                # remove WP at current_seq  
                self.wps.waypoints.pop(self.wps.current_seq)    
                succ = self.wpPush()
                return succ
            if remSeq < len(self.wps.waypoints):
                # remove WP at remSeq
                self.wps.waypoints.pop(remSeq)
                succ = self.wpPush()
        return succ
                          
    # *** CLEAR_MISSION ***
    # Remove all waypoints on FCU WP list
    def clearMission(self): 
        succ = False

        # Instead of calling mavros/mission/clear service, which also clears home wp,
        # just clear local list and add current coordinates as new home
        del self.wps.waypoints[:]

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
           
            succ = self.wpPush()
                
        return succ

    # *** BACKWARDS_MISSION ***
    # Invert WP list after home position (excluding [0])
    def invertWPlist(self): 
        succ = False

        if(len(self.wps.waypoints) > 2):
            self.wps.waypoints[1:] = self.wps.waypoints[len(self.wps.waypoints):0:-1]
        
        #Send list to the FCU
        succ = self.wpPush()
        return succ

    # *** OFFSET_MISSION ***
    # Add new WP as the last item on the list, using current GPS location
    def offsetWPlist(self, all_wps, offSeq, distance, direction): 
        succ = False

        if not all_wps:
            if (len(self.wps.waypoints) > offSeq):
                self.wps.waypoints[offSeq] = self.offsetWP(self.wps.waypoints[offSeq], distance, direction)
        
        elif (len(self.wps.waypoints) > 1):
            wp_list = WaypointList()
            wp_list.waypoints.append(self.wps.waypoints[0])
            for wpoff in self.wps.waypoints[1:]:
                wpoff = self.offsetWP(wpoff, distance, direction)
                wp_list.waypoints.append(wpoff)
            self.wps.waypoints = wp_list.waypoints

        succ = self.wpPush()
        return succ
        
    def offsetWP(self, wpset, dist, dxion):
        if (not self.k_set) and self.got_gp:
            self.calculateCoefficients()

        #rospy.loginfo("offsetWP: lat {0} long{1}".format(wpset.x_lat, wpset.y_long))
        k_x = math.sin(math.radians(dxion))
        k_y = math.cos(math.radians(dxion))
        #rospy.loginfo("OFFSET dist: {0} dxion: {1} k_y: {2} k_x: {3}".format(dist, dxion, k_y, k_x))
        distX = dist * k_x / self.k_long
        distY = dist * k_y / self.k_lat
        #rospy.loginfo("OFFSET distY: {0} distX: {1}".format(distY, distX))
        wpset.y_long += distX
        wpset.x_lat += distY
        #rospy.loginfo("now lat {0} long{1}".format(wpset.x_lat, wpset.y_long))
        return wpset
        
    # *** SCALE_ROTATE_MISSION ***
    # Move each WP away from Home, measured by distance * sFactor, rotating it in relation to Home
    def scaleRotateMission(self, sFactor, offsetAngle):
        succ = False
        vector = distVector()
        rospy.loginfo("sFactor: {0}, offsetAngle: {1}".format(sFactor, offsetAngle))
        midpoint = self.calculateMidpoint()
        rospy.loginfo("Midpoint x_lat {0} - y_long{1}".format(midpoint.x_lat, midpoint.y_long))

        if (len(self.wps.waypoints) > 1):
            for wpoff in self.wps.waypoints[1:]:
                rospy.loginfo("WPalku - x_lat: {0}, y_long: {1}".format(wpoff.x_lat, wpoff.y_long))
            wp_list = WaypointList()
            wp_list.waypoints.append(self.wps.waypoints[0])
            
            for wpoff in self.wps.waypoints[1:]:
                #rospy.loginfo("Home - y_long: {0}, x_lat: {1}".format(self.wps.waypoints[0].y_long, self.wps.waypoints[0].x_lat))
                rospy.loginfo("WPoff - x_lat: {0}, y_long: {1}".format(wpoff.x_lat, wpoff.y_long))
                vector = self.calculateVector(midpoint, wpoff)
                rospy.loginfo("vector.dist: {0}, vector.dxion: {1}".format(vector.dist, vector.dxion))
                wpoff.x_lat = midpoint.x_lat
                wpoff.y_long = midpoint.y_long
                #rospy.loginfo("E - y_long: {0}, x_lat: {1}".format(wpoff.y_long, wpoff.x_lat))
                wpoff = self.offsetWP(wpoff, (vector.dist * sFactor), (vector.dxion + offsetAngle))
                #rospy.loginfo("J - y_long: {0}, x_lat: {1}".format(wpoff.y_long, wpoff.x_lat))
                wp_list.waypoints.append(wpoff)
                rospy.loginfo(len(wp_list.waypoints))
            self.wps.waypoints = wp_list.waypoints
            succ = self.wpPush()

        return succ

    # *** MIRROR_MISSION ***
    # Move each WP rotating it in relation to Home, maintaining distance
    def mirrorMission(self, mirrorAngle):
        succ = False
        oVector = distVector()
        midpoint = self.calculateMidpoint()
        
        if (len(self.wps.waypoints) > 2):
        # Determine mission mindpoint

            # mirror axle heading must be in range [-90, 90]
            while mirrorAngle > 90:
                mirrorAngle -= 90
            while mirrorAngle < -90:
                mirrorAngle += 90
            
            # direction of moving each mirrored WP
            offsetAngle = mirrorAngle + 90    
                                                            
            wp_list = WaypointList()
            wp_list.waypoints.append(self.wps.waypoints[0])

            # process each waypoint
            for wpoff in self.wps.waypoints[1:]:
                # hypotenuse of triangle ABC wpoff - midpoint - (the point where the mirror offset vector crosses the mirror axle)
                oVector = self.calculateVector(wpoff, midpoint)
                # angle BAC
                alpha = math.radians(offsetAngle - oVector.dxion)
                # side AC
                halfDist = math.cos(alpha) * oVector.dist
                # move WP to mirrored location
                wpoff = self.offsetWP(wpoff, (2 * halfDist), offsetAngle)
                wp_list.waypoints.append(wpoff)
            self.wps.waypoints = wp_list.waypoints
            succ = self.wpPush()

        return succ

        
#    def nullF(self)
#        return False

    # *******************************************************************************************************
    # Check requested task
    def handle_task(self, req):
        MissionManipResponse()
        success = False

        rospy.wait_for_service('mavros/mission/pull')
        try:
            wpPullService = rospy.ServiceProxy('mavros/mission/pull', WaypointPull)
            res = wpPullService()
            if res.success:
                rospy.loginfo(res.wp_received)

        except rospy.ServiceException as e:
            rospy.loginfo("Service call pull waypoint failed: %s"%e)

        # set home in current location
        if req.task == 0:
            success = self.setCurrentHome()

        # add WP
        if req.task == 1:
            success = self.amendLocalWP(req.use_last, req.use_current, req.seq)

        # remove WP
        if req.task == 2:
            success = self.removeCurrentWP(req.use_last, req.use_current, req.seq)

        # clear mission
        if req.task == 3:
            success = self.clearMission()

        # backwards mission
        if req.task == 4:
            success = self.invertWPlist()

        # offset mission
        if req.task == 5:
            success = self.offsetWPlist(req.all_wps, req.seq, req.distance, req.direction_angle)

        # scale & rotate mission
        if req.task == 6:
            success = self.scaleRotateMission(req.scale_factor, req.direction_angle)

        # mirror mission
        if req.task == 7:
            success = self.mirrorMission(req.direction_angle)

        # Request actual number of WPs from the FCU
        number_wps = 0

        rospy.wait_for_service('mavros/mission/pull')
        try:
            wpPullService = rospy.ServiceProxy('mavros/mission/pull', WaypointPull)
            res = wpPullService()
            if res.success:
                number_wps = res.wp_received

        except rospy.ServiceException as e:
            rospy.loginfo("Service call pull waypoint failed: %s"%e)
            
        # Return service response message
        return MissionManipResponse(success, number_wps)    
    # *******************************************************************************************************

    # *** Calculation functions ***
    # Calculate distance and bearing for a distVector object
    def calculateVector(self, wpSource, wpDestination):
        resultVector = distVector()

        if (not self.k_set) and self.got_gp:
            self.calculateCoefficients()

        # here x = longitude, y = latitude
        diffy = (wpDestination.x_lat - wpSource.x_lat)
        diffx = (wpDestination.y_long - wpSource.y_long)
        
        disty = diffy * self.k_lat
        distx = diffx * self.k_long
        resultVector.dist = math.sqrt(disty**2 + distx**2)
               
        t_bearing = 0
        try:
            if resultVector.dist > 0:
                theta = math.acos(disty/resultVector.dist)
                t_bearing = math.degrees(theta)

                if distx < 0:
                    t_bearing = 360 - t_bearing
            resultVector.dist = round(resultVector.dist,2)

            resultVector.dxion = t_bearing
        except rospy.ServiceException as e:
            rospy.loginfo("Direction calculation failed: %s"%e)
                
        return resultVector
        
    # Calculate good-enough approximates
    def calculateCoefficients(self):
        # adjusted equatorial diameter to compensate differences between the geoid and a sphere, for  lat ~63
        diagEq = 12793172 
        self.k_lat = math.pi * diagEq / 360
        try:
            self.k_long = math.pi * math.cos(math.radians(self.global_position.latitude)) * diagEq / 360
        except rospy.ServiceException as e: 
            rospy.loginfo("Calculating k_long: %s"%e)

        self.k_set = True
        rospy.set_param('autopilot/k_lat', self.k_lat)
        rospy.set_param('autopilot/k_long', self.k_long)
                    
    # Calculate distance & bearing to next WP
    def distanceNextWP(self):
        if (not self.k_set) and self.got_gp:
            self.calculateCoefficients()

        nextwp = 1
        if (self.wps.current_seq < len(self.wps.waypoints)) and (self.wps.current_seq > 0):
            nextwp = self.wps.current_seq
        # here x = longitude, y = latitude
        diffy = (self.wps.waypoints[nextwp].x_lat - self.global_position.latitude)
        diffx = (self.wps.waypoints[nextwp].y_long - self.global_position.longitude)
        
        disty = (diffy * self.k_lat)
        distx = (diffx * self.k_long)
        dist = math.sqrt(disty**2 + distx**2)
        
        self.wpinfo_msg.distance_to_next_wp = dist
        
        t_bearing = 0
        try:
            if dist > 0:
                #rospy.loginfo("Disty: {0}, dist: {1}".format(disty,dist))
                theta = math.acos(disty/dist)
                t_bearing = math.degrees(theta)

                if distx < 0:
                    t_bearing = 360 - t_bearing
            dist = round(dist,2)
            #rospy.loginfo("dist: {0}, bearing: {1}".format(dist,t_bearing))
            self.wpinfo_msg.target_bearing = t_bearing
        except rospy.ServiceException as e:
            rospy.loginfo("Bearing calculation failed: %s"%e)

    # **********************************************************************************************

    def calculateMidpoint(self):
        midpoint = Waypoint()
        midpoint.x_lat = self.wps.waypoints[1].x_lat
        midpoint.y_long = self.wps.waypoints[1].y_long
        
        if (len(self.wps.waypoints) > 2):
        # Determine mission mindpoint
            maxX = midpoint.y_long
            minX = midpoint.y_long
            maxY = midpoint.x_lat
            minY = midpoint.x_lat
            
            for wp in self.wps.waypoints[2:]:
                maxX = max(maxX, wp.y_long)
                minX = min(minX, wp.y_long)
                maxY = max(maxY, wp.x_lat)
                minY = min(minY, wp.x_lat)
            midpoint.y_long = (minX + maxX) / 2
            midpoint.x_lat = (minY + maxY) / 2
        return midpoint
        
    # *** Run function ***
    def run(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            if len(self.wps.waypoints) > 1:
                self.distanceNextWP()
                self.pub_wpinfo.publish(self.wpinfo_msg)
            rate.sleep()

    # *** Callback functions ***
    # Read WP list from the FCU
    def callback_mission_wps(self, data): 
        if self.wps.current_seq != data.current_seq:
            rospy.loginfo("Current mission waypoint sequence updated: {0}".format(data.current_seq))
        self.wps = data
        self.wpinfo_msg.total_wps = len(self.wps.waypoints)

        self.wpinfo_msg.next_wp = self.wps.current_seq

    # Read GPS position data
    def callback_global_position(self, data): 
        self.global_position = data
        self.got_gp = True

def shutdown_msg():
    rospy.loginfo("Exiting server node")    

if __name__ == "__main__":
    rospy.on_shutdown(shutdown_msg)
    wpservice = WPmanip()
    wpservice.run()
