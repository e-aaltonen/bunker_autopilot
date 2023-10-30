#!/usr/bin/python

"""
dashboard.py
Esa Aaltonen 2023

This is an ad-hoc feedback monitor for developing purposes by simply echoing
/wp_info published by wp_amend_clear.py and /angles published by dashboard.py 
in pop-up terminal windows (GNOME Terminal).
 
Requirements:
- wp_amend_clear.py running
- MavROS node (mavros_node) running

"""
import rospy
from std_msgs.msg import UInt8
from sensor_msgs.msg import Imu
from autopilot_test.msg import Angles
import time
import subprocess
import math

class Dash():
    def __init__(self):
        rospy.init_node("dashboard")
        self.sub_imu = rospy.Subscriber("mavros/imu/data", Imu, self.callback_imu)
        rospy.on_shutdown(killViews)
        
        self.pub_angles = rospy.Publisher("angles", Angles, queue_size=1)

        self.angles_msg = Angles()

        # Simple WP dashboard
        cmd = "gnome-terminal --geometry 30x6+50+0 --hide-menubar --zoom 1.2 --command 'rostopic echo /wp_info'"
        subprocess.call(cmd, shell=True) 
        
        # Simple Angles dashboard
        cmd = "gnome-terminal --geometry 30x4+50+300 --hide-menubar --zoom 1.2 --command 'rostopic echo /angles'"
        subprocess.call(cmd, shell=True) 

    def killViews(self):
        nodes = os.popen("rosnode list").readlines()
        for i in range(len(nodes)):
        nodes[i] = nodes[i].replace("\n","")
        
        print (nodes)

        for node in nodes:

            if node[1:9] == "rostopic":
                os.system("rosnode kill "+ node)
                
        nodes = os.popen("rosnode list").readlines()        
        print (nodes)
        
    def callback_imu(self, msg):
        xacc = msg.linear_acceleration.x
        yacc = msg.linear_acceleration.y
        zacc = msg.linear_acceleration.z

        aPitch = math.degrees(math.atan(xacc / zacc))
        aRoll = math.degrees(math.atan(yacc / zacc))
        self.angles_msg.pitch = aPitch
        self.angles_msg.roll = aRoll

    def pwm_to_adimensional(self, pwm):
        pwm = max(pwm, self._pwm_min)
        pwm = min(pwm, self._pwm_max)
        pwm = pwm - ((self._pwm_max + self._pwm_min) * 0.5)
        pwm = pwm / ((self._pwm_max - self._pwm_min) * 0.5)
        return pwm

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.pub_angles.publish(self.angles_msg)
            rate.sleep()
    

if __name__ == "__main__":
    d_board = Dash()
    d_board.run()

