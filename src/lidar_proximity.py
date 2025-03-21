#!/usr/bin/python3

import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32
import struct
import math
import time

class LidarProx():
    def __init__(self):
        rospy.init_node("lidar_proximity")

        # Robot dimensions
        self.path_width_per_side = 0.60 / 2 # robot width / 2 (default for Scout Mini: 0.60 / 2)
        self.floor_level = -0.40 # height relative to lidar horizontal level (default for Scout Mini: -0.45)
        self.robot_front_edge = 0.2 # distance from lidar origo to robot front edge (default for Scout Mini: 0.2)
        self.robot_rear_edge = -0.4 # distance from lidar origo to robot rear edge (default for Scout Mini: -0.4)

        # Behaviour ranges
        self.stop_range_lateral = 0.2 + self.path_width_per_side # stop when detecting objects parallel to trajectory closer than this (default: 0.5 + self.path_width_per_side)
        self.slowdown_range_lateral = 0.1 + self.stop_range_lateral # slow down when detecting objects parallel to trajectory closer than this (default: 1.0 + self.path_width_per_side)
        self.stop_range_front = 1.0 + self.robot_front_edge # stop when detecting object directly in front of the robot closer than this (default: 1.0 + self.robot_front_dimension)
        self.slowdown_range_front = 2.5 + self.stop_range_front # slow down when detecting object directly in front of the robot closer than this (default: 3.0 + self.robot_front_dimension)
        self.max_slope_coefficient = self.slope_coefficient(15) # corresponds to an elevation of 15 degrees
        self.proximity_speed = 1.0  # cruise speed factor to be published for the autopilot system
        self.proximity_speed_previous = 1.0

        # Filter pointcloud2 messages & data
        self.incoming_rate = 1 # only process 1 of n pointcloud2 messages (default: 3)
        self.incoming_counter = 0
        self.rings = 5 # process n lowermost channels

        # Check params
        if rospy.has_param('autopilot/robot_width'):
            self.path_width_per_side = rospy.get_param('autopilot/robot_width') / 2

        if rospy.has_param('autopilot/floor_level'):
            self.floor_level = rospy.get_param('autopilot/floor_level')

        if rospy.has_param('autopilot/robot_front_edge'):
            self.robot_front_edge = rospy.get_param('autopilot/robot_front_edge')
        
        if rospy.has_param('autopilot/robot_rear_edge'):
            self.robot_rear_edge = rospy.get_param('autopilot/robot_rear_edge')
        
        if rospy.has_param('autopilot/stop_range_lateral'):
            self.stop_range_lateral = rospy.get_param('autopilot/stop_range_lateral') + self.path_width_per_side
        
        if rospy.has_param('autopilot/slowdown_range_lateral'):
            self.slowdown_range_lateral = rospy.get_param('autopilot/slowdown_range_lateral') + self.stop_range_lateral

        if rospy.has_param('autopilot/stop_range_front'):
            self.stop_range_front = rospy.get_param('autopilot/stop_range_front') + self.robot_front_edge

        if rospy.has_param('autopilot/slowdown_range_front'):
            self.slowdown_range_front = rospy.get_param('autopilot/slowdown_range_front') + self.stop_range_front
        
        if rospy.has_param('autopilot/max_slope'):
            self.max_slope_coefficient = self.slope_coefficient(rospy.get_param('autopilot/max_slope'))

        if rospy.has_param('autopilot/incoming_rate'):
            self.incoming_rate = rospy.get_param('autopilot/incoming_rate')

        if rospy.has_param('autopilot/rings'):
            self.rings = rospy.get_param('autopilot/rings')

        self._lidar_topic = "/velodyne_points"
        if rospy.has_param('autopilot/lidar_topic'):
            self._lidar_topic = rospy.get_param('autopilot/lidar_topic')

        self._proximity_points_topic = "/autopilot/proximity_points"
        if rospy.has_param('autopilot/proximity_topic'):
            self._proximity_points_topic = rospy.get_param('autopilot/proximity_topic')

        self._proximity_speed_topic = "/autopilot/proximity_speed"
        if rospy.has_param('autopilot/proximity_speed_topic'):
            self._proximity_speed_topic = rospy.get_param('autopilot/proximity_speed_topic')

        self.pc2_msg = rospy.Subscriber(self._lidar_topic, PointCloud2, self.parse_pc2)
        rospy.loginfo("> Subscriber for lidar points: {0}".format(self._lidar_topic))
        self.pc2_msg_publisher = rospy.Publisher("/pub_points", PointCloud2, queue_size=1)
        rospy.loginfo("> Publisher for proximity points: {0}".format(self._lidar_topic))
        self.pub_msg = PointCloud2()
        self.proximity_speed_publisher = rospy.Publisher(self._proximity_speed_topic, Float32, queue_size=1)

    def slope_coefficient(self, slope):
        theta = math.radians(90-slope)

        # Lowest 2 laser rings
        alpha0 = math.radians(15)
        alpha1 = math.radians(13)
        
        res = (math.tan(theta) * math.sin(alpha0) - math.tan(theta) * math.sin(alpha1)) / (math.tan(theta) * math.sin(alpha1) + 1)

        return res

    def slope_ahead(self, max_x_values, min_x_values):
        is_slope = False

        valid_scan_lines = 0

        rings = 0
        if len(max_x_values) == len(min_x_values):
            rings = len(max_x_values)

        for i in range(rings):
            if max_x_values[i] < 99.0 and min_x_values[i] > 0.0:
                valid_scan_lines = valid_scan_lines + 1

        if valid_scan_lines > 2:
            is_slope = True

        for i in range(rings-1):
            if max_x_values[i] > min_x_values[i+1]:
                # x value ranges between rings are overlapping: texture too rough
                is_slope = False
            if max_x_values[i] < 99.0 and min_x_values[i+1] > 0.0 and (min_x_values[i+1] - min_x_values[i]) / min_x_values[i] < self.max_slope_coefficient:
                # too steep a slope is a wall
                is_slope = False
        return is_slope
    
    def parse_pc2(self, msg):
        # Process only 1 of (self.incoming_rate = 3) of incoming velodyne_points messages
        self.incoming_counter += 1

        if self.incoming_counter >= self.incoming_rate:
            self.incoming_counter = 0

            self.pub_msg.header = msg.header
            self.pub_msg.height = msg.height
            #self.pub_msg.width = msg.width
            self.pub_msg.fields = msg.fields
            self.pub_msg.is_bigendian = msg.is_bigendian
            self.pub_msg.point_step = msg.point_step
            self.pub_msg.row_step = msg.row_step
            self.pub_msg.is_dense = msg.is_dense
            self.pub_msg.data = b''

            scanPoints = int(len(msg.data) / msg.point_step)
            
            min_x_rings = [99.9] * self.rings # minimum x value per ring of points detected directly in front of the robot
            max_x_rings = [0.0] * self.rings # maximum x value per ring of points detected directly in front of the robot
            nearest_object_lateral = 99.9
            nearest_object_ahead = 99.9

            flag_slowdown_side = False  # scanned points within the lateral slow-down zone: published in /pub_points
            flag_slowdown_front = False # points within the front slow-down zone
            flag_stop = False           # scanned points within the stop zone: close-by in front or lateral to tthe trajectory
            flag_slope = False          # low-lying points ahead interpreted as a slope: OK to proceed
            flag_low_obstacle = False   # low-lying points ahead interpreted as an obstacle - these will not be detected at close range: not OK to proceed

            # Point x, y, z coordinates must be expressed as float32
            if not (msg.fields[0].datatype == 7 and msg.fields[1].datatype == 7 and msg.fields[2].datatype == 7):
                rospy.logerr("Expecting float32 datatype (enum 7) for x, y, z. Incoming messages are using {0}, {1}, {2}".format(msg.fields[0].datatype, msg.fields[1].datatype, msg.fields[2].datatype ))
            
            # loop through all data points
            for scanpoint_index in range(scanPoints):
                if msg.data[msg.point_step * scanpoint_index + 16] < self.rings:
                    # Convert 4-byte sequences into float values for x, y, z
                    byte_array_x = bytearray(b'')
                    byte_array_y = bytearray(b'')
                    byte_array_z = bytearray(b'')
                    
                    for byte_index in range(4):
                        byte_array_x.append(msg.data[msg.point_step * scanpoint_index + byte_index])
                        byte_array_y.append(msg.data[msg.point_step * scanpoint_index + byte_index + 4])
                        byte_array_z.append(msg.data[msg.point_step * scanpoint_index + byte_index + 8])

                    float_value_x = struct.unpack('f', byte_array_x)[0]
                    float_value_y = struct.unpack('f', byte_array_y)[0]
                    float_value_z = struct.unpack('f', byte_array_z)[0]

                    # narrow down data points
                    # is point above floor level, not behind the robot's rear edge
                    if (float_value_z > self.floor_level and float_value_x > self.robot_rear_edge):
                        
                        # is point in slow-down zone (between lateral slowdown_range perimeters, behind front slowdown_range)
                        if (abs(float_value_y) < self.slowdown_range_lateral and float_value_x < self.slowdown_range_front):
                            # flag slow-down
                            flag_slowdown_side = True
                            object_distance = math.sqrt(float_value_x**2 + float_value_y**2)

                            # is the point in the parallel sector or on the trajctory
                            if (abs(float_value_y > self.stop_range_lateral)):
                                nearest_object_lateral = min(nearest_object_lateral, object_distance)  
                            if (float_value_x > self.stop_range_front):       
                                nearest_object_ahead = min(nearest_object_ahead, object_distance)                       

                            # populate message to be published: all points in slow-down zone
                            for pub_byte_index in range(msg.point_step):
                                byte_to_add = msg.data[msg.point_step * scanpoint_index + pub_byte_index].to_bytes(1, byteorder='big')
                                self.pub_msg.data += byte_to_add

                            # is point on trajectory
                            if (abs(float_value_y) < (self.stop_range_lateral) and float_value_x > self.robot_front_edge):
                                # determine min/max x values per ring
                                flag_slowdown_front = True
                                current_ring = msg.data[msg.point_step * scanpoint_index + 16]
                                min_x_rings[current_ring] = min(min_x_rings[current_ring], float_value_x)
                                max_x_rings[current_ring] = max(max_x_rings[current_ring], float_value_x)

                                # is point in stop zone (on trajectory or in proximity on either side)
                                if (float_value_x < self.stop_range_front):
                                    flag_stop = True   

            self.pub_msg.width = int(len(self.pub_msg.data)/self.pub_msg.point_step)

            flag_slope = self.slope_ahead(max_x_rings, min_x_rings)

            if (flag_slope):
                flag_slowdown_front = False
                rospy.loginfo("Rinne!")

            if ((flag_slowdown_front or flag_slowdown_side) and not flag_stop and not flag_slope):
                rospy.loginfo("Hidasta!")
                if (nearest_object_ahead < self.slowdown_range_front):
                    #rospy.loginfo("Este edessä: {0}".format(nearest_object_ahead))
                    self.proximity_speed = (nearest_object_ahead - self.stop_range_front) / (self.slowdown_range_front - self.stop_range_front)
                if (nearest_object_lateral < self.slowdown_range_lateral):
                    #rospy.loginfo("Este sivulla: {0}".format(nearest_object_lateral))
                    self.proximity_speed = min(self.proximity_speed, ((nearest_object_lateral - self.stop_range_lateral) / (self.slowdown_range_lateral - self.stop_range_lateral)))

            if flag_stop:
                rospy.loginfo("Seis!")
                self.proximity_speed = 0.0

            if (not flag_stop and not flag_slowdown_side and not flag_slowdown_front):
                self.proximity_speed = 1.0
            
            # Publish new proximity speed value if updated
            if self.proximity_speed != self.proximity_speed_previous:
                self.publish_proximity_speed()
                self.proximity_speed_previous = self.proximity_speed
            
            #rospy.loginfo("Min x 0: {0:.3f}, x 1: {1:.3f}, x 2: {2:.3f}, x 3: {3:.3f}, x 4: {4:.3f}".format(min_x_rings[0], min_x_rings[1], min_x_rings[2], min_x_rings[3], min_x_rings[4]))
    
    def publish_proximity_speed(self):
        # send updated speed factor according to detected objects in the slowdown zone
        self.proximity_speed_publisher.publish(self.proximity_speed)

    def run(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.pc2_msg_publisher.publish(self.pub_msg)
            rate.sleep()
    

if __name__ == "__main__":
    node_item = LidarProx()
    node_item.run()
