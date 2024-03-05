#!/usr/bin/python

"""
rcout_to_cmd_vel.py
E. Aaltonen 2024

Convert skid steering servo signal to Twist messages

This node reads FCU skid steering throttle output from /mavros/rc/out and publishes corresponding Twist values
to /cmd_vel.
Channel[0]: throttle left  (SERVO1_FUNCTION = 73)
Channel[2]: throttle right (SERVO3_FUNCTION = 74)

Requirements:
- CAN up (sudo ip link set can0 up type can bitrate 500000)
- bunker_bringup running (bunker_minimal.launch)
- MAVROS node running (/mavros/mavros_node)

Default parameter values set in /autopilot_test/launch/autopilot_test_agx.launch:
- autopilot/speed_factor_lin_x: "1.0"   Forward movement rate under FCU control
- autopilot/speed_factor_ang_y: "1.0"   Turning rate under FCU control
"""

import rospy
from std_msgs.msg import UInt16
from geometry_msgs.msg import Twist
from mavros_msgs.msg import RCOut
import time

class RCOutInput():
    def __init__(self):
        rospy.init_node("rcout_to_cmd_vel")
        self.sub_rc = rospy.Subscriber("mavros/rc/out", RCOut, self.update_throttle)
        rospy.loginfo("> Subscriber for /mavros/rc/out created")

        self._pwm_min = 900
        self._pwm_max = 2100
       
        self._speed_factor_lin_x = 1.0
        self._speed_factor_ang_z = 1.0

        if rospy_has_param('autopilot/speed_factor_lin_x'):
            self._speed_factor_lin_x = rospy.get_param('autopilot/speed_factor_lin_x')
        if rospy_has_param('autopilot/speed_factor_ang_y'):
            self._speed_factor_ang_y = rospy.get_param('autopilot/speed_factor_ang_y')
                
        self.pub_twist = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        self.twist_msg = Twist()

    def update_throttle(self, msg):
        self.twist_msg.linear.x = self.pwm_to_adimensional((msg.channels[0] + msg.channels[2]) * 0.5) * self._speed_factor_lin_x
        self.twist_msg.angular.z = self.pwm_to_adimensional(msg.channels[2]) - self.pwm_to_adimensional(msg.channels[0]) * self._speed_factor_ang_z

    def pwm_to_adimensional(self, pwm):
        pwm = max(pwm, self._pwm_min)
        pwm = min(pwm, self._pwm_max)
        pwm = pwm - ((self._pwm_max + self._pwm_min) * 0.5)
        pwm = pwm / ((self._pwm_max - self._pwm_min) * 0.5)
        return pwm

    def run(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            self.pub_twist.publish(self.twist_msg)
            rate.sleep()
    

if __name__ == "__main__":
    rc_proxy = RCOutInput()
    rc_proxy.run()

