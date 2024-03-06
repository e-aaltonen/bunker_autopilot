#!/usr/bin/python
"""
rc_state_messenger.py
E. Aaltonen 2024

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
in UInt8 format, 0 = up, 1 = middle, 2 = down, and in topic /switch/var_a, Int8 -100 ... 100

Requirements:
- CAN up (sudo ip link set can0 up type can bitrate 500000)
- bunker_bringup running (bunker_robot_base.launch) (modified bunker_ros pkg)

"""

import rospy
from std_msgs.msg import UInt8, Int8
from bunker_msgs.msg import BunkerRCState
from bunker_autopilot.msg import RCchannels


# int literals for switch positions
SW_UP = 0
SW_MIDDLE = 1
SW_DOWN = 2

class SWMessenger():
    def __init__(self):
        rospy.init_node("rc_state_sub_pub")
        
        # Default topic when using Bunker
        self._rc_topic = "bunker_rc_status"
        if rospy.has_param('autopilot/rc_topic'):
            self._rc_topic = rospy.get_param('autopilot/rc_topic')
                    
        self.sub = rospy.Subscriber(self._rc_topic, BunkerRCState, self.callback_rc_status)

        rospy.loginfo("> Subscriber created: RC switches messenger")

        self.pub_swa = rospy.Publisher("autopilot/switch/a", UInt8, queue_size=1)
        self.pub_swb = rospy.Publisher("autopilot/switch/b", UInt8, queue_size=1)
        self.pub_swc = rospy.Publisher("autopilot/switch/c", UInt8, queue_size=1)
        self.pub_swd = rospy.Publisher("autopilot/switch/d", UInt8, queue_size=1)
        self.pub_var_a = rospy.Publisher("autopilot/switch/var_a", Int8, queue_size=1)
        self.pub_channels = rospy.Publisher("autopilot/RCchannels", RCchannels, queue_size=1)
        self.channel_msg = RCchannels()
        self.channel_msg.button = 0

        self.last_swa = 0
        self.last_swb = 0
        self.last_swc = 0
        self.last_swd = 0
        self.last_var_a = 0
        
    def callback_rc_status(self, msg):
        self.channel_msg.right_x = msg.stick_right_h
        self.channel_msg.right_y = msg.stick_right_v
        self.channel_msg.left_x = msg.stick_left_h
        self.channel_msg.left_y = msg.stick_left_v

        # Check if any of the switches have changed and publish in topics autopilot/switch/a, b, c or d
        # Only publish if switch value is 0, 1 or 2 (= eliminate rubbish values during 2-pos. switch transition)
        if msg.swa != self.last_swa and msg.swa in range(3):
            self.pub_swa.publish(msg.swa)
            self.last_swa = msg.swa

        if msg.swb != self.last_swb and msg.swb in range(3):
            self.pub_swb.publish(msg.swb)
            self.last_swb = msg.swb

        if msg.swc != self.last_swc and msg.swc in range(3):
            self.pub_swc.publish(msg.swc)
            self.last_swc = msg.swc

        if msg.swd != self.last_swd and msg.swd in range(3):
            self.pub_swd.publish(msg.swd)
            self.last_swd = msg.swd

        # Check if Left knob has changed and publish in topic autopilot/switch/var_a
        if msg.var_a != self.last_var_a:
            self.pub_var_a.publish(self.last_var_a)  
            self.last_var_a = msg.var_a
        
        self.channel_msg.swa = self.sw_to_chan(msg.swa)
        self.channel_msg.swb = self.sw_to_chan(msg.swb)
        self.channel_msg.swc = self.sw_to_chan(msg.swc)
        self.channel_msg.swd = self.sw_to_chan(msg.swd)
                        
            
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

