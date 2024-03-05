#!/usr/bin/python3

# E. Aaltonen 2024

import rospy
from std_msgs.msg import UInt16, UInt8, Int8
from mavros_msgs.msg import RCIn, ParamValue
from mavros_msgs.srv import ParamGet, ParamSet
from bunker_autopilot.msg import RCchannels
import time
import math

# int literals - switch positions
SW_UP = 0
SW_MIDDLE = 1
SW_DOWN = 2

class RCChannel:
    def __init__(self, chnl):
        self.minpwm = 1000
        self.maxpwm = 2000
        self.trim = 1500
        self.dz = 0
        self.channel = chnl
        
        self.attrlist = [self.minpwm, self.maxpwm, self.trim, self.dz]
        self.paramnames = ["_MIN", "_MAX", "_TRIM", "_DZ"]
        
        for i in range(3):
            rospy.wait_for_service('mavros/param/get')
            paramIDtext = self.channel + self.paramnames[i]
            try:
                paramService = rospy.ServiceProxy('mavros/param/get', ParamGet)
                result = paramService(param_id = paramIDtext)
                if result.success:
                    self.attrlist[i] = result.value.integer
                    rospy.loginfo("Received param {0} - value {1}".format(paramIDtext, result.value.integer))
            except rospy.ServiceException as e:
                rospy.loginfo("Service call failed: %s"%e)
    
class RCInMessenger():
    def __init__(self):
        rospy.init_node("rcin_messenger")
        self.channel_msg = RCchannels()
       
        # label RC channels
        self.right_x = RCChannel("RC1")
        self.right_y = RCChannel("RC2")
        self.left_x = RCChannel("RC3")
        self.left_y = RCChannel("RC4")
        self.swa = RCChannel("RC5")
        self.swb = RCChannel("RC6")
        self.var_a = RCChannel("RC9")
        self.button = RCChannel("RC10")
                        
        self.pub_channels = rospy.Publisher("autopilot/RCchannels", RCchannels, queue_size=1)

        self.pub_swa = rospy.Publisher("autopilot/switch/a", UInt8, queue_size=1)
        self.pub_swb = rospy.Publisher("autopilot/switch/b", UInt8, queue_size=1)
        self.pub_swc = rospy.Publisher("autopilot/switch/c", UInt8, queue_size=1)
        self.pub_swd = rospy.Publisher("autopilot/switch/d", UInt8, queue_size=1)
        self.pub_var_a = rospy.Publisher("autopilot/switch/var_a", Int8, queue_size=1)
        self.pub_button = rospy.Publisher("autopilot/switch/button", UInt8, queue_size=1)

        self.channelsNew = [0, 0, 0, 0, 0, 0]  # PWM values 1000-2000
        self.channelsPrev = [0, 0, 0, 0, 0, 0] # PWM values 1000-2000

        self.sub_rc = rospy.Subscriber("mavros/rc/in", RCIn, self.cb_rcin)

    def mapSwitch(self, pwm):
        swPos = SW_MIDDLE
        if pwm < 1200:
            swPos = SW_UP
        if pwm > 1800:
            swPos = SW_DOWN
        return swPos
    
    def mapVar(self, pwm):
        return int((pwm - 1500) / 5)

    def cb_rcin(self, msg):    
        if len(msg.channels) > 8:
            self.channel_msg.right_x = self.scale_pwm(msg.channels[0], self.right_x)
            self.channel_msg.right_y = self.scale_pwm(msg.channels[1], self.right_y)
            self.channel_msg.left_x = self.scale_pwm(msg.channels[3], self.left_x)
            self.channel_msg.left_y = self.scale_pwm(msg.channels[2], self.left_y)
            self.channel_msg.swa = self.scale_pwm(msg.channels[4], self.swa)
            self.channel_msg.swb = self.scale_pwm(msg.channels[5], self.swb)
            self.channel_msg.var_a = self.scale_pwm(msg.channels[8], self.var_a)
            self.channel_msg.button = self.scale_pwm(msg.channels[9], self.button)

            self.channelsNew[0] = self.mapSwitch(msg.channels[4])   # SWA
            self.channelsNew[1] = self.mapSwitch(msg.channels[5])   # SWB
            self.channelsNew[2] = self.mapSwitch(msg.channels[7])   # SWC
            self.channelsNew[3] = self.mapSwitch(msg.channels[6])   # SWD
            self.channelsNew[4] = self.mapVar(msg.channels[8])  # var_a
            self.channelsNew[5] = self.mapSwitch(msg.channels[9])  # button
        
    
        #Check if any of the switches have changed and publish in topics /switch/a, b, c, d, var_a, button
        if self.channelsNew != self.channelsPrev:
            if self.channelsNew[0] != self.channelsPrev[0]:
                self.pub_swa.publish(self.channelsNew[0])
            if self.channelsNew[1] != self.channelsPrev[1]:
                self.pub_swb.publish(self.channelsNew[1])
                self.set_servos(self.channelsNew[1])
            if self.channelsNew[2] != self.channelsPrev[2]:
                self.pub_swc.publish(self.channelsNew[2])
            if self.channelsNew[3] != self.channelsPrev[3]:
                self.pub_swd.publish(self.channelsNew[3])
            if self.channelsNew[4] != self.channelsPrev[4]:
                self.pub_var_a.publish(self.channelsNew[4])
            if self.channelsNew[5] != self.channelsPrev[5]:
                self.pub_button.publish(self.channelsNew[4])
            for x in range(6):
                self.channelsPrev[x] = self.channelsNew[x]

    # If switch (B) is down, disable throttle & steering servo output to enable GUI; else enable servos
    def set_servos(self, sw):
        paramService = rospy.ServiceProxy('mavros/param/set', ParamSet)
        disable = ParamValue()
        disable.integer = 0
        enable_steering = ParamValue()
        enable_steering.integer = 26
        enable_throttle = ParamValue()
        enable_throttle.integer = 70
        param_id_1 = "SERVO1_FUNCTION"
        param_id_3 = "SERVO3_FUNCTION"
                
        # disable
        if sw == SW_DOWN:
            rospy.wait_for_service('mavros/param/set')
            try:
                result = paramService(param_id_1, disable)
                if result.success:
                    rospy.loginfo("Disabled Servo 1")
                result = paramService(param_id_3, disable)
                if result.success:
                    rospy.loginfo("Disabled Servo 3")
            except rospy.ServiceException as e:
                rospy.loginfo("Service call failed: %s"%e)
        else:
            rospy.wait_for_service('mavros/param/set')
            try:
                result = paramService(param_id_1, enable_steering)
                if result.success:
                    rospy.loginfo("Enabled Servo 1")
                result = paramService(param_id_3, enable_throttle)
                if result.success:
                    rospy.loginfo("Enabled Servo 3")
            except rospy.ServiceException as e:
                rospy.loginfo("Service call failed: %s"%e)
            
    def scale_pwm(self, inpwm, rcChan):
        #rospy.loginfo("inpwm: {0}, rcChan.maxpwm: {1}, rcChan.minpwm: {2}, rcChan.trim: {3}".format(inpwm, rcChan.maxpwm, rcChan.minpwm, rcChan.trim))
        inpwm = min(inpwm, rcChan.maxpwm)
        inpwm = max(inpwm, rcChan.minpwm)
        if inpwm > rcChan.trim:
            rate = (inpwm - rcChan.trim) / (rcChan.maxpwm - rcChan.trim) * 100
        else:
            rate = (inpwm - rcChan.trim) / (rcChan.trim - rcChan.minpwm) * 100
        if math.fabs(rate - rcChan.trim) < rcChan.dz:
            rate = 0
        #rospy.loginfo("RC {0}: {1} -> {2}".format(rcChan.channel, inpwm, rate))
        return int(rate)

    def run(self):
        rate = rospy.Rate(25)
        while not rospy.is_shutdown():
            self.pub_channels.publish(self.channel_msg)
            rate.sleep()
 
if __name__ == "__main__":
    rc_proxy = RCInMessenger()
    rc_proxy.run()

