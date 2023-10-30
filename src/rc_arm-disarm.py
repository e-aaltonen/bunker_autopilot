#!/usr/bin/python
"""
rc_arm-disarm.py
Esa Aaltonen 2023

Read remote control switches C & D to control FCU flight mode (C) and arming/disarming (D)

This node reads RC switch states from /switch/c and /switch/d and calls flight mode service
/mavros/set_mode to set MANUAL mode (switch C up) or AUTO mode (switch D middle) 
and arming command service /mavros/cmd/arming to arm (switch D down) or disarm (switch D up) the FCU.

Requirements:
- CAN up (sudo ip link set can0 up type can bitrate 500000)
- bunker_bringup running (bunker_minimal.launch) (modified code)
- MavROS node running (/mavros/mavros_node)
- rc_state_sub-pub.py running (publisher for /switch/c and /switch/d)

"""

import rospy
from std_msgs.msg import UInt8, Int8
from mavros_msgs.srv import CommandBool, SetMode
import time

# int literals - switch positions
SW_UP = 2
SW_MIDDLE = 1
SW_DOWN = 3


class RCArming():
    def __init__(self):
        rospy.init_node("rc_arm_disarm")
        self.sub_swc = rospy.Subscriber("switch/c", UInt8, self.callback_update_swc)
        self.sub_swd = rospy.Subscriber("switch/d", UInt8, self.callback_update_swd)

        rospy.loginfo("> Subscriber created: arm/disarm")

        self.swc = 0
        self.swd = 0

    # Read Switch C value
    def callback_update_swc(self, msg):

        #Check if switch C state has changed
        if msg.data != self.swc:
            self.swc = msg.data
            if msg.data == SW_UP: # 2 = switch up - call manual
                rospy.wait_for_service('mavros/set_mode')
                try:
                    flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
                    isModeChanged = flightModeService(custom_mode='MANUAL') #return true or false
                except rospy.ServiceException as e:
                    rospy.loginfo("Set mode service call failed: %s"%e)
                
            if msg.data == SW_MIDDLE: # 1 = switch middle - call auto
                rospy.wait_for_service('mavros/set_mode')
                try:
                    flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
                    isModeChanged = flightModeService(custom_mode='AUTO') #return true or false
                except rospy.ServiceException as e:
                    rospy.loginfo("Set mode service call failed: %s"%e)
                

    # Read Switch D value
    def callback_update_swd(self, msg):
        #Check if switch D state has changed
        if msg != self.swd:
            self.swd = msg.data
            if msg.data == SW_DOWN: # 3 = switch down call arm
                rospy.wait_for_service('mavros/cmd/arming')
                try:
                    armService = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
                    armService(True)
                    rospy.loginfo("Arming service call successful")
                except rospy.ServiceException as e:
                    rospy.loginfo("Arming service call failed: %s"%e)
                
            if msg.data == SW_UP: # call disarm
                rospy.wait_for_service('mavros/cmd/arming')
                try:
                    armService = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
                    armService(False)
                    rospy.loginfo("Disarming service call successful")
                except rospy.ServiceException as e:
                    rospy.loginfo("Disarming service call failed: %s"%e)

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            rate.sleep()



if __name__ == "__main__":
    arm_proxy = RCArming()
    arm_proxy.run()

