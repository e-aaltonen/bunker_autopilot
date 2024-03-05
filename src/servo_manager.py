#!/usr/bin/python
"""
servo_manager.py
E. Aaltonen 2024

Subscribe to autopilot/switch/var_a and autopilot/switch/button and manage the state of 3 servos accordingly.
Servo PWM signals are set in parameters SERVOn_TRIM (to be used by MAVROS & ArduRover as output for Pixhawk RCOUT ports)

"""

import rospy
from std_msgs.msg import UInt8, Int8
from mavros_msgs.msg import ParamValue
from mavros_msgs.srv import CommandBool, SetMode
import time

# int literals - switch positions
SW_UP = 0
SW_MIDDLE = 1
SW_DOWN = 2

F_PARAM = "SERVO6_TRIM"
R_PARAM = "SERVO7_TRIM"
SPEED_PARAM = "SERVO8_TRIM"


class ServoMngr():
    def __init__(self):
        rospy.init_node("servo_manager")
        rospy.Subscriber("autopilot/switch/var_a", Int8, self.cb_var_a)
        rospy.Subscriber("autopilot/switch/button", UInt8, self.cb_button)

        rospy.loginfo("> Initiating servo_manager")

        self.var_a = 0
        self.button = 0
        self.front_lock = false
        self.f_pwm = [ 1900, 1100 ]
        self.rear_lock = false
        self.r_pwm = [ 1100, 1900 ]
        self.speed_high = false
        self.sp_pwm = [ 1100, 1900 ]

    # Read Switch C value
    def cb_allback_update_swc(self, msg):

        #Check if button state has changed
        if msg.data != self.button:
            self.button = msg.data
            if msg.data == SW_DOWN:
                # left knob turned left (down): toggle rear diff
                if self.var_a < -25:
                    self.rear_lock = not self.rear_lock
                    self.set_servos(R_PARAM, self.r_pwm[self.rear_lock])
                
                # left knob turned right (up): toggle front diff
                if self.var_a > 25: 
                    self.front_lock = not self.front_lock
                    self.set_servos(F_PARAM, self.f_pwm[self.front_lock])
                
                # left knob in the middle: toggle speed
                if (self.var_a > -26) & (self.var_a < 26): 
                    self.speed_high = not self.speed_high
                    self.set_servos(SPEED_PARAM, self.sp_pwm[self.speed_high])
               
    def set_servos(self, param_id, param_value):
        paramService = rospy.ServiceProxy('mavros/param/set', ParamSet)

        p_value = ParamValue()
        p_value.integer = param_value

        param_id = pname
                
        rospy.wait_for_service('mavros/param/set')
        try:
            result = paramService(param_id, p_value)
            if result.success:
                rospy.loginfo("Sent value {0} to param {1}".format(param_value, param_id))
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s"%e)
                
    # Read left knob value
    def cb_var_a(self, msg): 
        self.var_a = msg.data

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            rate.sleep()



if __name__ == "__main__":
    arm_proxy = ServoMngr()
    arm_proxy.run()

