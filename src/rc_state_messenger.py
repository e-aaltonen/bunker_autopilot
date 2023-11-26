#!/usr/bin/python
"""
rc_state_messenger.py
Esa Aaltonen 2023

Read remote control data feed and publish messages at switch state change
Also publish stick values [-100,100] and switch values [0,100] in topic /autopilot/RCchannels (for GUI)

This node subscribes to /bunker_rc_status (cf. BunkerRCState.msg) published by bunker_messenger.cpp
(uint8 sws, int8 var_a), cf. Bunker Pro user manual, p. 10:

uint8 sws, switches: CAN frame 0x241, byte [0]:
bit[0-1]: SWA 2 = up, 3 = down
bit[2-3]: SWB 2 = up, 1 = middle, 3 = down
bit[4-5]: SWC 2 = up, 1 = middle, 3 = down
bit[6-7]: SWD 2 = up, 3 = down

int8 var_a "left knob": CAN frame 0x241, byte[5]:
range [-100,100]: -100 = left/down limit, 0 = middle, 100 = right/up limit

When a value changes, the node publishes the corresponding new state once in topic /switch/a, /b, /c or /d
in UInt8 format, 2 = up, 1 = middle, 3 = down, and in topic /switch/var_a, Int8 -100 ... 100

Requirements:
- CAN up (sudo ip link set can0 up type can bitrate 500000)
- bunker_bringup running (bunker_minimal.launch) (modified code)

"""

import rospy
from std_msgs.msg import UInt8, Int8
from bunker_msgs.msg import BunkerRCState
from bunker_autopilot.msg import RCchannels

# int literals - switch positions
SW_UP = 2
SW_MIDDLE = 1
SW_DOWN = 3

class SWMessenger():
    def __init__(self):
        rospy.init_node("rc_state_sub_pub")
        self.sub = rospy.Subscriber("bunker_rc_status", BunkerRCState, self.callback_rc_status)

        rospy.loginfo("> Subscriber created: RC switches messenger")

        self.pub_swa = rospy.Publisher("switch/a", UInt8, queue_size=1)
        self.pub_swb = rospy.Publisher("switch/b", UInt8, queue_size=1)
        self.pub_swc = rospy.Publisher("switch/c", UInt8, queue_size=1)
        self.pub_swd = rospy.Publisher("switch/d", UInt8, queue_size=1)
        self.pub_var_a = rospy.Publisher("switch/var_a", Int8, queue_size=1)
        self.pub_channels = rospy.Publisher("autopilot/RCchannels", RCchannels, queue_size=1)
        self.channel_msg = RCchannels()
        self.channel_msg.button = 0

        self.last_sws = 0
        self.sws_last = [0, 0, 0, 0]
        self.last_var_a = 0
        self.sw_array = [0, 0, 0, 0]        
        
    def callback_rc_status(self, msg):
        self.channel_msg.right_x = msg.right_stick_left_right
        self.channel_msg.right_y = msg.right_stick_up_down
        self.channel_msg.left_x = msg.left_stick_left_right
        self.channel_msg.left_y = msg.left_stick_up_down
        self.channel_msg.swa = self.scale_pwm(msg.channels[4], self.swa)
        self.channel_msg.swb = self.scale_pwm(msg.channels[5], self.swb)
        self.channel_msg.var_a = self.scale_pwm(msg.channels[8], self.var_a)
        self.channel_msg.button = self.scale_pwm(msg.channels[9], self.button)

        #Check if any of the switches have changed and publish in topics /switch/a, b, c or d
        if msg.sws != self.last_sws:
            
            for j in range(4):
                k = 3 - j
                mask = (3 << (k*2))
                sw_value = (msg.sws & mask) >> (k*2)
                self.sw_array[k] = sw_value

            self.channel_msg.swa = self.sw_to_chan(self.sw_array[0])
            self.channel_msg.swb = self.sw_to_chan(self.sw_array[1])
            self.channel_msg.swc = self.sw_to_chan(self.sw_array[2])
            self.channel_msg.swd = self.sw_to_chan(self.sw_array[3])
	    
            if(self.sw_array[0] != self.sws_last[0]):
                self.pub_swa.publish(self.sw_array[0])
            if(self.sw_array[1] != self.sws_last[1]):
                self.pub_swb.publish(self.sw_array[1])
            if(self.sw_array[2] != self.sws_last[2]):
                self.pub_swc.publish(self.sw_array[2])
            if(self.sw_array[3] != self.sws_last[3]):
                self.pub_swd.publish(self.sw_array[3])  

            self.last_sws = msg.sws
                        
        # Check if Left knob has changed and publish in topic /switch/var_a
        if msg.var_a != self.last_var_a:
            self.pub_var_a.publish(self.last_var_a)  
            self.last_var_a = msg.var_a
    
    def sw_to_chan(self, pos):
        chan = 0
        if pos == 1: chan = 50
        if pos == 3: chan = 100
        return chan

    def run(self):
        rate = rospy.Rate(25)
        while not rospy.is_shutdown():
            self.pub_channels.publish(self.channel_msg)
            rate.sleep()



if __name__ == "__main__":
    sw_proxy = SWMessenger()
    sw_proxy.run()

