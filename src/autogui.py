#!/usr/bin/python3

# E. Aaltonen 5 Mar 2024

import rospy
import time
import tkinter as tk
from tkinter import ttk
from std_msgs.msg import Int8, UInt8, Float64
from std_msgs.msg import String
from mavros_msgs.msg import State, Waypoint, WaypointList
from sensor_msgs.msg import Imu
from bunker_autopilot.srv import MissionManip
from bunker_autopilot.msg import RCchannels, WPInfo
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped
import math
import pyautogui
import sys


# int literals - mission server tasks
SET_HOME = 0
ADD_WP = 1
REMOVE_WP = 2
CLEAR_MISSION = 3
BACKWARDS_MISSION = 4
OFFSET_MISSION = 5
SCALE_MISSION = 6
ROTATE_MISSION = 7

# int literals - switch positions
SW_UP = 0
SW_MIDDLE = 1
SW_DOWN = 2

class LFrame(tk.LabelFrame):
    def __init__(self, *args, **kwargs):
        tk.LabelFrame.__init__(self, *args, **kwargs)
        self['padx'] = 10
        self['pady'] = 10
        #self['bd'] = 1
        self['highlightthickness'] = 3
        #e.widget['highlightcolor'] = ('gray95')
        self['highlightbackground'] = ('gray95')
        self.bind("<Enter>", self.on_enter)
        self.bind("<Leave>", self.on_leave)

    def on_enter(self, e):
        e.widget['foreground'] = ('blue')
        #e.widget['highlightcolor'] = ('blue')
        e.widget['highlightbackground'] = ('white')
    def on_leave(self, e):
        e.widget['foreground'] = ('black')        
        #e.widget['highlightcolor'] = ('gray95')
        e.widget['highlightbackground'] = ('gray95')

class ServiceButton(tk.Button):
    def __init__(self, *args, **kwargs):
        tk.Button.__init__(self, *args, **kwargs)
        self['width'] = 14
        self['width'] = kwargs.get('width')
        self['padx'] = 3        
        self['pady'] = 18
        
class ServiceFrame(tk.Frame):
    def __init__(self, *args, **kwargs):
        tk.Frame.__init__(self, *args, **kwargs)
        self['highlightbackground'] = "gray95"
        self['highlightthickness'] = 3
        self['bg'] = kwargs.get('bg')
        self['padx'] = 3
        self['pady'] = 3
        
class ListWP():
    def __init__(self, x_lat, y_long):
        self.x_lat = x_lat
        self.y_long = y_long
        self.x_m = 0.0
        self.y_m = 0.0
        self.selected = False
        
class Ohjelma():
    def __init__(self):
        pyautogui.move(100, 100, duration = 0.5)
        
        self.global_position = NavSatFix()  
        self.global_position.latitude=62.123456
        self.global_position.longitude=23.123456

        self.swa = 0
        self.swb = 0
        
        self.button = 0
        self.task = 0
        self.use_last = True
        self.use_current = False
        self.seq = 0
        self.all_wps = False

        self.distance = 0.0
        self.direction_angle = 0.0
        self.scale_factor = 1.0
        
        self.bearing = 180
        self.speed = 1.2
        self.roll = 45
        self.pitch = 67
        
        self.modeArm = "Disarmed"
        self.modeCtrl = "Manual"
        self.prevArm = ""
        self.prevCtrl = ""
        self.label_arm = tk.Label(root, text=self.modeArm, fg="red")
        self.label_mod = tk.Label(root, text=self.modeCtrl, fg="brown")
        
        self.lang = tk.StringVar()
        self.langindex = 0
        
        self.service_button_texts = [["Add", "Remove", "Clear", "Backwards", "Offset", "Scale/rotate", "Mirror"], ["Lisää", "Poista", "Tyhjennä", "Takaperin", "Siirrä", "Skaalaa/kierrä", "Peilaa"], ["Tillägg", "Radera", "Tömma", "Bakåt", "Förskjut", "Skala/vrid", "Spegla"]]
        self.f_wps_text = ["Mission waypoints", "Reittipisteet", "Vägpunkterna"]
        self.wp_listbox_0_text = ["All waypoints", "Kaikki reittipisteet", "Alla vägpunkter"]
        self.label_cur_tot_text = ["Next: ", "Seur. ", "Nästa: "]
        self.f_attr_text = ["Parameters", "Parametrit", "Parametrar"]
        self.var_text = [ ["last WP", "next WP", "WP ", "all WPs"], ["viim. rp", "seur. rp", "rp ", "kaikki rp:t"], ["sista rp", "nästa rp", "rp ", "alla rp:er"] ]
        self.r_use_texts = ["Apply to ", "Kohde: ", "Anv. till "]
        self.f_dist_text = ["Distance", "Etäisyys", "Avstånd"]
        self.f_scale_text = ["Scale factor", "Skaalauskerroin", "Skalningsfaktor"]
        self.f_dir_text = ["Direction", "Suunta", "Riktning"]
        self.button_reset_text = ["Reset", "Nollaa", "Återställ"]
        self.r_use_text_labels = [["last", "next", "selected", "all"], ["viimeinen", "seuraava", "valittu", "kaikki"], ["sista", "nästa", "vald", "alla"]]

        # Frame for service attributes
        self.f_attr = LFrame(root, bd=4, text=self.f_attr_text[self.langindex])    # LFrame
        self.f_attr.grid(row=1, rowspan=2, column=2, sticky="N")
        

        if len(sys.argv)>1:
            try:
                lgs = ["en", "fi", "sv"]
                for i in range(len(lgs)):
                    if sys.argv[1] == lgs[i]:
                        self.langindex = i                        
            except rospy.ServiceException as e:
                rospy.loginfo("Language code must 'en', 'fi' or 'sv': %s"%e)

        self.langmenu = ttk.Combobox(root, width=8, textvariable=self.lang)
        self.langmenu['values'] = ('English', 'suomi', 'svenska')
        self.langmenu.grid(row=0, column=0, pady=(15,25)) # ROOT
        #self.langmenu.current(self.langindex)
        self.langmenu.current(self.langindex)

        self.f_buttons = tk.Frame(root)
        self.frame_add = ServiceFrame(self.f_buttons, bg="#E61717") #52
        self.button_add = ServiceButton(self.frame_add, text=self.service_button_texts[self.langindex][0], activebackground="#E67373", command=lambda: self.service_mission(ADD_WP))
        self.button_add.pack()#padx=10, pady=10
        self.frame_rem = ServiceFrame(self.f_buttons, bg="#E6CA17") #52
        self.button_rem = ServiceButton(self.frame_rem, text=self.service_button_texts[self.langindex][1], activebackground="#E6D673", command=lambda: self.service_mission(REMOVE_WP))
        self.button_rem.pack()
        self.frame_clr = ServiceFrame(self.f_buttons, bg="#4EE617") #104
        self.button_clr = ServiceButton(self.frame_clr, text=self.service_button_texts[self.langindex][2], activebackground="#92E673", command=lambda: self.service_mission(CLEAR_MISSION))
        self.button_clr.pack()
        self.frame_bkw = ServiceFrame(self.f_buttons, bg="#17E693") #156
        self.button_bkw = ServiceButton(self.frame_bkw, text=self.service_button_texts[self.langindex][3], activebackground="#73E6B8", command=lambda: self.service_mission(BACKWARDS_MISSION))
        self.button_bkw.pack()
        self.frame_off = ServiceFrame(self.f_buttons, bg="#1785E6") #208
        self.button_off = ServiceButton(self.frame_off, text=self.service_button_texts[self.langindex][4], activebackground="#73B0E6", command=lambda: self.service_mission(OFFSET_MISSION))
        self.button_off.pack()
        self.frame_rot = ServiceFrame(self.f_buttons, bg="#5B17E6") #260
        self.button_rot = ServiceButton(self.frame_rot, text=self.service_button_texts[self.langindex][5], activebackground="#9973E6", command=lambda: self.service_mission(SCALE_MISSION))
        self.button_rot.pack()
        self.frame_mir = ServiceFrame(self.f_buttons, bg="#E617BD") #312
        self.button_mir = ServiceButton(self.frame_mir, text=self.service_button_texts[self.langindex][6], activebackground="#E673CF", command=lambda: self.service_mission(ROTATE_MISSION))
        self.button_mir.pack()
        
        self.k_lat = 111194
        self.k_long = 50519
        if rospy.has_param('autopilot/k_lat') and rospy.has_param('autopilot/k_long'):
            self.k_lat = rospy.get_param('autopilot/k_lat')
            self.k_long = rospy.get_param('autopilot/k_long')

        self.total_range = 0    # Size of mission in metres
        
        self.wpinfo_data = WPInfo()
        
        self.current = 0
        self.wplist = []

        self.set_local_coordinates()

        # Frame for sensor data
        self.f_sensor = LFrame(root, bd=2)
        self.label_bearing_legend = tk.Label(self.f_sensor, text="⟠", font=('TkDefaultFont', 16))
        self.label_bearing = tk.Label(self.f_sensor, text=str(self.bearing), font=('TkDefaultFont', 18))
        
        self.label_speed = tk.Label(self.f_sensor, text=str(self.speed) + " m/s", font=('TkDefaultFont', 14))
        self.label_pitch_legend = tk.Label(self.f_sensor, text="↕")
        self.label_pitch = tk.Label(self.f_sensor, text=str(self.pitch) + "°")
        self.label_roll_legend = tk.Label(self.f_sensor, text="↔") 
        self.label_roll = tk.Label(self.f_sensor, text=str(self.roll) + "°")

        self.label_pos_legend = tk.Label(self.f_sensor, text="⌖", font=('TkDefaultFont', 18))
        self.label_xlat = tk.Label(self.f_sensor, text=str(round(self.global_position.latitude,6)) + " N")
        self.label_ylong = tk.Label(self.f_sensor, text=str(round(self.global_position.longitude,6)) + " E")
        
        self.label_bearing_legend.grid(row=0, column=0)
        self.label_bearing.grid(row=0, column=1, sticky="E")
        self.label_speed.grid(row=0, column=3, sticky="E")
        self.label_pitch_legend.grid(row=2, column=0)
        self.label_pitch.grid(row=2, column=1, sticky="E")
        self.label_roll_legend.grid(row=2, column=2, padx=(15,0))
        self.label_roll.grid(row=2, column=3)
        self.label_pos_legend.grid(row=3, rowspan=2, column=0, pady=20)
        self.label_xlat.grid(row=3, column=1, columnspan=3, sticky="S", pady=(10, 0))
        self.label_ylong.grid(row=4, column=1, columnspan=3, sticky="N", pady=0)

        self.f_sensor.grid(row=1, column=0)    # ROOT
       
        # Frame for waypoint list
        self.f_wps = LFrame(root, bd=4, text=self.f_wps_text[self.langindex]) # LFrame
        self.f_wps.grid(row=1, rowspan=2, column=1, padx=(0,15), sticky="N")  # ROOT

        self.wplist_scrollup = tk.Label(self.f_wps, text="↑")
        self.wplist_scrolldown = tk.Label(self.f_wps, text="↓")
        self.label_wp = tk.Label(self.f_wps, text="No. (X, Y) m")        

 
        self.label_wp.grid(row=0, column=1)
               
        self.wp_listbox = tk.Listbox(self.f_wps, bg="black", fg="yellow", height=20)
        
        self.wp_listbox.grid(row=1, column=0, columnspan=2)


        #self.label_next = tk.Label(self.f_wps, text="Next: waypoint")        
        self.label_cur_tot = tk.Label(self.f_wps, text=self.label_cur_tot_text[self.langindex] + str(self.current) + "/" + str(len(self.wplist)))

        self.wplist_scrollup.grid(row=0, column=0)
        self.wplist_scrolldown.grid(row=2, column=0)

        #self.label_next.grid(row=2, column=1)
        self.label_cur_tot.grid(row=3, column=1)
        
        # Initialise f_attr xy position
        self.f_attr_x = 0
        self.f_attr_y = 0
        
        # Initialise wp_listbox xy position
        self.wp_listbox_x = 0
        self.wp_listbox_y = 0
        
        self.var_bool = tk.IntVar()

        self.r_use_last = tk.Radiobutton(self.f_attr, text=self.r_use_text_labels[self.langindex][0], variable=self.var_bool, value=0, command=lambda: self.rsel(-1))
        self.r_use_last.select()
        self.r_use_curr = tk.Radiobutton(self.f_attr, text=self.r_use_text_labels[self.langindex][1], variable=self.var_bool, value=1, command=lambda: self.rsel(self.current))        
        self.r_use_sel = tk.Radiobutton(self.f_attr, text=self.r_use_text_labels[self.langindex][2], state="disabled", variable=self.var_bool, value=2, command=lambda: self.rsel(0))
        self.r_use_all = tk.Radiobutton(self.f_attr, text=self.r_use_text_labels[self.langindex][3], variable=self.var_bool, value=3, command=lambda: self.rsel(0))

        self.r_use_text = tk.Label(self.f_attr, text=self.r_use_texts[self.langindex] + self.var_text[self.langindex][0], width=20)

        self.r_use_last.grid(row=0, column=1, columnspan=3, sticky="W")
        self.r_use_curr.grid(row=1, column=1, columnspan=3, sticky="W")
        self.r_use_sel.grid(row=2, column=1, columnspan=3, sticky="W")
        self.r_use_all.grid(row=3, column=1, columnspan=3, sticky="W")
        
        self.r_use_text.grid(row=4, column=0, columnspan=4, pady = (10,30))
        
        self.separator1 = tk.Label(self.f_attr, bd=2, relief=tk.RAISED, text=" ")
        self.separator1.place(relx=0, rely=0.32, relwidth=1, relheight=0.008)
        #self.separator1.grid(row=4, column=0, columnspan=4)
        self.separator2 = tk.Label(self.f_attr, bd=2, relief=tk.RAISED, text=" ")
        self.separator2.place(relx=0.2, rely=0.65, relwidth=0.6, relheight=0.005)
        
        self.separator3 = tk.Label(self.f_attr, bd=2, relief=tk.RAISED, text=" ")
        self.separator3.place(relx=0.2, rely=0.91, relwidth=0.6, relheight=0.005)
        
        self.f_dist = LFrame(self.f_attr, text=self.f_dist_text[self.langindex], borderwidth=0)      # LFrame
        self.f_scale = LFrame(self.f_attr, text=self.f_scale_text[self.langindex], borderwidth=0) # LFrame
        self.f_dir = LFrame(self.f_attr, text=self.f_dir_text[self.langindex], borderwidth=0)      # LFrame

        self.f_dist.grid(row=5, column=1, columnspan=2, sticky="W")
        self.f_scale.grid(row=7, column=1, columnspan=2, sticky="W", pady=(0,25))
        self.f_dir.grid(row=9, column=1, columnspan=2, sticky="W")
        
        self.entry_dist = tk.StringVar()
        self.entry_dist.trace("w", self.on_entry_dist_change)
        self.e_distance_1 = tk.Label(self.f_dist, text = "", width=2)
        self.e_distance = tk.Entry(self.f_dist, width=6, borderwidth=2, textvariable=self.entry_dist)
        self.e_distance_label = tk.Label(self.f_dist, text = "m", width=4)

        self.entry_sfactor = tk.StringVar()
        self.entry_sfactor.trace("w", self.on_entry_sfactor_change)
        self.e_scale_factor = tk.Entry(self.f_scale, width=5, borderwidth=2, textvariable=self.entry_sfactor)
        self.e_sfactor_label = tk.Label(self.f_scale, text = "k:", width=2)

        self.entry_dir = tk.StringVar()
        self.entry_dir.trace("w", self.on_entry_dir_change)
        self.e_direction_1 = tk.Label(self.f_dir, text = "", width=4)
        self.e_direction = tk.Entry(self.f_dir, width=4, borderwidth=2, textvariable=self.entry_dir)
        self.e_direction_label = tk.Label(self.f_dir, text = "°")
        
        self.e_distance_1.grid(row=6, column=1)
        self.e_distance.grid(row=6, column=2)
        self.e_distance_label.grid(row=6, column=3)
        self.e_scale_factor.grid(row=8, column=2)
        self.e_sfactor_label.grid(row=8, column=1, sticky="E")
        self.e_direction_1.grid(row=10, column=1)
        self.e_direction.grid(row=10, column=2)
        self.e_direction_label.grid(row=10, column=3, sticky="W")        
        
        self.e_distance.insert(0, str(self.distance))
        self.e_scale_factor.insert(0, str(self.scale_factor))
        self.e_direction.insert(0, str(self.direction_angle))

        self.f_updown = tk.Frame(self.f_attr)
        self.button_up = tk.Button(self.f_updown, text="⏶", padx=3, pady=0, width=1, height=2, command=lambda: self.button_updown(1))
        self.button_down = tk.Button(self.f_updown, text="⏷", padx=3, pady=0, width=1, height=2,  command=lambda: self.button_updown(-1))
        self.f_leftright = tk.Frame(self.f_attr)
        self.button_left = tk.Button(self.f_leftright, text="⏴", padx=0, pady=0, width=4, height=1, command=lambda: self.button_leftright(-1))
        self.button_right = tk.Button(self.f_leftright, text="⏵", padx=0, pady=0, width=4, height=1, command=lambda: self.button_leftright(1))
        self.button_reset = tk.Button(self.f_attr, text=self.button_reset_text[self.langindex], padx=0, pady=0, width=6, height=1, command=self.button_reset)

        self.f_updown.grid(row=5, rowspan=3)
        self.button_up.grid(row=0, column=0)
        self.button_down.grid(row=1, column=0)

        self.f_leftright.grid(row=11, column=0, columnspan=4)
        self.button_left.grid(row=0, column=0)
        self.button_right.grid(row=0, column=1)
        self.button_reset.grid(row=12, column=0, columnspan=4, pady=(25,0))

        self.wp_map = tk.Canvas(root, width=250, height=250, background='midnight blue')
        self.wp_map.grid(row=2, column=0, padx=15, pady=(10, 20))  # ROOT  
        self.draw_map()
        
        self.sep_rainbow1 = tk.Label(root, bd=0, highlightbackground='blue', highlightthickness=1)
        self.sep_rainbow1.place(relx=0.10, rely=0.08, relwidth=0.9, relheight=0.001)
        self.sep_rainbow2 = tk.Label(root, bd=0, highlightbackground='blue', highlightthickness=1)
        self.sep_rainbow2.place(relx=0.12, rely=0.084, relwidth=0.9, relheight=0.001)

        # Frame for output messages
        self.f_output = tk.Frame(root)
        self.f_output.grid(row=3, column=0, columnspan=4)
        self.output_label = tk.Label(self.f_output, text="")
        self.output_label.pack()
        
        
        # Event bindings:
        self.langmenu.bind("<<ComboboxSelected>>", self.setlang)
        self.e_distance.bind("<Button-4>", lambda x: self.button_updown(1))
        self.e_distance.bind("<Button-5>", lambda x: self.button_updown(-1))

        self.e_scale_factor.bind("<Button-4>", lambda x: self.button_updown(1))
        self.e_scale_factor.bind("<Button-5>", lambda x: self.button_updown(-1))

        self.e_direction.bind("<Button-4>", lambda x: self.button_leftright(1))
        self.e_direction.bind("<Button-5>", lambda x: self.button_leftright(-1))
                
        self.wp_listbox.bind("<<ListboxSelect>>", self.event_listbox_select)
        self.wp_listbox.bind("<Button-4>", lambda x: self.update_scroll_arrows(1))  
        self.wp_listbox.bind("<Button-5>", lambda x: self.update_scroll_arrows(-1))  

        self.wps = WaypointList()
        
        rospy.Subscriber("/mavros/state", State, self.cb_state)
        rospy.Subscriber("/mavros/imu/data", Imu, self.cb_imu)
        rospy.Subscriber("/mavros/mission/waypoints", WaypointList, self.cb_mission_wps)
        rospy.Subscriber("/autopilot/RCchannels", RCchannels, self.cb_cursor)
        rospy.Subscriber("/wp_info", WPInfo, self.cb_wpinfo)
        rospy.Subscriber("/mavros/global_position/compass_hdg", Float64, self.cb_compass_hdg)
        rospy.Subscriber("/mavros/local_position/velocity_body", TwistStamped, self.cb_velocity)

        self.sub_swa = rospy.Subscriber("switch/a", UInt8, self.cb_swa)
        self.sub_swb = rospy.Subscriber("switch/b", UInt8, self.cb_swb)
        self.sub_global_pos = rospy.Subscriber("mavros/global_position/global", NavSatFix, self.cb_global_position)      

    # ***   ***     ***
    #Tk bound events callback functions
    def on_entry_dist_change(self, *args):
        new_value = self.entry_dist.get()
        try:
            self.distance = float(new_value)
        except:
            pass

    def setlang(self, *args):
        rospy.loginfo("kieli")
        rospy.loginfo(str(self.lang.get()) + str(self.langmenu.current()))
        self.langmenu.selection_clear()
        self.langindex = self.langmenu.current()
        self.f_wps.config(text=self.f_wps_text[self.langindex])

        sel_index = self.var_bool.get()
                        
        self.wp_listbox.delete(0)
        self.wp_listbox.insert(0, self.wp_listbox_0_text[self.langindex])
        if self.seq > 0:
            self.wp_listbox.select_set(self.seq-1)
        if self.use_current:
            self.wp_listbox.select_set(self.current)
        if self.use_last:
            self.wp_listbox.select_set(len(self.wplist)-1)
        
        self.label_cur_tot.config(text=self.label_cur_tot_text[self.langindex] + str(self.current) + "/" + str(len(self.wplist)))               
        self.f_attr.config(text=self.f_attr_text[self.langindex]) 
        self.r_use_text.config(text=self.r_use_texts[self.langindex] + self.var_text[self.langindex][0])
        self.f_dist.config(text=self.f_dist_text[self.langindex])
        self.f_scale.config(text=self.f_scale_text[self.langindex])
        self.f_dir.config(text=self.f_dir_text[self.langindex])
        self.button_reset.config(text=self.button_reset_text[self.langindex])
        self.button_add.config(text=self.service_button_texts[self.langindex][0])
        self.button_clr.config(text=self.service_button_texts[self.langindex][2])
        self.button_bkw.config(text=self.service_button_texts[self.langindex][3])
        self.button_off.config(text=self.service_button_texts[self.langindex][4])
        self.button_rot.config(text=self.service_button_texts[self.langindex][5])
        self.button_mir.config(text=self.service_button_texts[self.langindex][6])
        self.r_use_last.config(text=self.r_use_text_labels[self.langindex][0])
        self.r_use_curr.config(text=self.r_use_text_labels[self.langindex][1])
        self.r_use_sel.config(text=self.r_use_text_labels[self.langindex][2])
        self.r_use_all.config(text=self.r_use_text_labels[self.langindex][3])
        self.var_bool.set(sel_index)

    def on_entry_sfactor_change(self, *args):
        new_value = self.entry_sfactor.get()
        try:
            self.scale_factor = float(new_value)
        except:
            pass

    def on_entry_dir_change(self, *args):
        new_value = self.entry_dir.get()
        try:
            self.direction_angle = float(new_value)
        except:
            pass 
                                              
    def update_attribute_frame_position(self):
        updtext = "x: " + str(self.f_attr_x) + ", y: " + str(self.f_attr_y)
        rospy.loginfo(updtext)

    def event_listbox_select(self, event):
        #self.wpsel.config(text = "WP no. " + str(self.wp_listbox.curselection()[-1]))
        self.var_bool.set(2)
        i = 0
        if len(self.wp_listbox.curselection()) > 0:
            i = self.wp_listbox.curselection()[-1]
        if i >= 0:        
            self.all_wps = False
            self.r_use_sel.config(state="normal")
            for wp in self.wplist:
                wp.selected = False
            self.wplist[i].selected = True
        """if i == 0:
            self.all_wps = True
            self.var_bool.set(3)
            for wp in self.wplist:
                wp.selected = True"""
        self.rsel(i)
    #***    ***     ***
    
    def update_scroll_arrows(self, direx):
        #rospy.loginfo("Yview: " + str(self.wp_listbox.yview()))
        if self.wp_listbox.yview()[0] > 0:
            self.wplist_scrollup.grid(row=0, column=0)
        else:
            self.wplist_scrollup.grid_forget()
        if self.wp_listbox.yview()[1] < 1:
            self.wplist_scrolldown.grid(row=2, column=0)
        else:
            self.wplist_scrolldown.grid_forget()
        
    def cursor_in_parameters_field(self):
        result = False
        x, y = pyautogui.position()
        self.f_attr_x = self.f_attr.winfo_rootx()
        self.f_attr_y = self.f_attr.winfo_rooty()

        endx = self.f_attr_x + self.f_attr.winfo_width()
        endy = self.f_attr_y + self.f_attr.winfo_height()
        if x >= self.f_attr_x and x < endx and y >= self.f_attr_y and y < endy:
            result = True    
        return result

    def cursor_in_wplist(self):
        result = False
        x, y = pyautogui.position()
        self.wp_listbox_x = self.wp_listbox.winfo_rootx()
        self.wp_listbox_y = self.wp_listbox.winfo_rooty()

        endx = self.wp_listbox_x + self.wp_listbox.winfo_width()
        endy = self.wp_listbox_y + self.wp_listbox.winfo_height()
        if x >= self.wp_listbox_x and x < endx and y >= self.wp_listbox_y and y < endy:
            result = True    
        return result

    def button_updown(self, increment):
        self.distance = round((self.distance + (increment / 10)),1)
        self.scale_factor = round((self.scale_factor + (increment / 10)),1)
        self.e_distance.delete(0, tk.END)
        self.e_distance.insert(0, str(self.distance))
        self.e_scale_factor.delete(0, tk.END)
        self.e_scale_factor.insert(0, str(self.scale_factor))

    def button_leftright(self, increment):
        self.direction_angle = round((self.direction_angle + (increment)),1)
        self.e_direction.delete(0, tk.END)
        self.e_direction.insert(0, str(self.direction_angle))
        
    def button_reset(self):
        self.distance = 0.0
        self.scale_factor = 1.0
        self.direction_angle = 0.0
        self.e_distance.delete(0, tk.END)
        self.e_distance.insert(0, str(self.distance))
        self.e_scale_factor.delete(0, tk.END)
        self.e_scale_factor.insert(0, str(self.scale_factor))
        self.e_direction.delete(0, tk.END)
        self.e_direction.insert(0, str(self.direction_angle))
        self.output_label.config(text = "")

    def service_mission(self, number):
        configtext = "task=" + str(number)
        configtext += ", use_last="
        if self.use_last:
            configtext += "True"
        else:
            configtext += "False"
        configtext += ", use_current="
        if self.use_current:
            configtext += "True"
        else:
            configtext += "False"
        configtext += ", seq="
        if self.seq > 0 and len(self.wp_listbox.curselection()) > 0:
            configtext += str(self.wp_listbox.curselection()[-1])
        configtext += ", all_wps="
        if self.all_wps:
            configtext += "True"
        else:
            configtext += "False"
        configtext += ", distance=" + str(self.distance)
        configtext += ", direction angle=" + str(self.direction_angle)
        configtext += ", scale_factor=" + str(self.scale_factor)
        rospy.loginfo("Call WP server: " + configtext)
        
        """if number == 2:
            self.wp_map.delete("all")
        """
        
        try:
            rospy.wait_for_service('mission_manip')
            wpService = rospy.ServiceProxy('mission_manip', MissionManip)
            serviceSuccess = wpService(task = number, use_last = self.use_last, use_current = self.use_current, seq = self.seq, all_wps = self.all_wps, distance = self.distance, direction_angle = self.direction_angle, scale_factor = self.scale_factor)
            self.output_label.config(text = configtext + " success: " + str(serviceSuccess))
        except rospy.ServiceException as e:
            rospy.loginfo("Service call mission_manip failed: %s"%e)
  
    def rsel(self, i):
        sel_index = self.var_bool.get()
        #self.var_list = [0, 0, 0, 0]
        #self.var_list[sel_index] = 1
        seltext = self.var_text[self.langindex][sel_index]
        rospy.loginfo("Sel_index {0}, i {1}".format(sel_index, i))

        #for wp in self.wplist:
        #    wp.selected = False

        """dd
        for i in range(4):
            if self.var_list[i] == 1:
                seltext = self.var_text[i]
        """
        if i != 0:
            for wp in self.wplist:
                wp.selected = False
            self.wplist[i].selected = True

        self.use_last = False
        self.use_current = False
        self.all_wps = False
        self.seq = 0        
        
        if sel_index != 2:
            self.wp_listbox.selection_clear(0, tk.END)
            self.r_use_sel.config(state="disabled")

        if sel_index == 0:  # use last
            self.use_last = True
            self.wp_listbox.select_set('end') #len(self.wplist)-1
            #rospy.loginfo(self.wp_listbox.curselection()[-1])
            self.wplist[-1].selected = True
        
        if sel_index == 1:  # use current from AUTO
            self.use_current = True
            self.wp_listbox.select_set(self.current)
            self.wplist[self.current].selected = True

        if sel_index == 2:  # use selected
            self.seq = (i+1)
            rospy.loginfo("Set self.seq = {0}".format(i))
            if len(self.wp_listbox.curselection()) > 0:
                seltext += str(self.wp_listbox.curselection()[-1])

        if sel_index == 3:  # use all
            self.all_wps = True
            self.wp_listbox.select_set(0)
            for wp in self.wplist:
                wp.selected = True
        self.r_use_text.config(text=self.r_use_texts[self.langindex] + seltext)
        self.draw_map()


    # *** Callback functions ***
    # Calculate pitch & roll angles
    def cb_imu(self, msg):
        xacc = msg.linear_acceleration.x
        yacc = msg.linear_acceleration.y
        zacc = msg.linear_acceleration.z

        aPitch = math.degrees(math.atan(xacc / zacc))
        aRoll = math.degrees(math.atan(yacc / zacc))
        self.pitch = round(aPitch,0)
        self.roll = round(aRoll,0)
        pitch_text = str(self.pitch) + "°"
        self.label_pitch.config(text=pitch_text)
        self.label_roll.config(text=str(self.roll) + "°")

    # Compass heading
    def cb_compass_hdg(self, msg):
        self.bearing = int(msg.data)
        self.label_bearing.config(text=str(self.bearing))
    
    # Linear velocity
    def cb_velocity(self, msg):
        self.speed = round(msg.twist.linear.x, 1)
        self.label_speed.config(text=str(self.speed) + " m/s")
    
    # FCU armed state & control state
    def cb_state(self, msg):
        if(msg.armed):
            self.modeArm = "Armed"
        else:
            self.modeArm = "Disarmed"

        self.modeCtrl = msg.mode

        if self.modeArm != self.prevArm:
            self.label_arm.config(text=self.modeArm)
            if self.modeArm == "Armed":
                self.label_arm.config(fg="red")
            else:
                self.label_arm.config(fg="green")
        self.prevArm = self.modeArm
        
        if self.modeCtrl != self.prevCtrl:
            self.label_mod.config(text=self.modeCtrl)
            if self.modeCtrl == "MANUAL":
                self.label_mod.config(fg="brown")
            else:
                self.label_mod.config(fg="blue")
        self.prevCtrl = self.modeCtrl

    # Read stick & button values from RCin
    def cb_cursor(self, msg):
        self.f_attr_x = self.f_attr.winfo_rootx()
        self.f_attr_y = self.f_attr.winfo_rooty()

        self.wp_listbox_x = self.wp_listbox.winfo_rootx()
        self.wp_listbox_y = self.wp_listbox.winfo_rooty()

        #self.update_attribute_frame_position()
        #self.update_mouse_position()     
        
        if self.swb == SW_DOWN:
            x = msg.right_x / 10
            y = msg.right_y / -10
            pyautogui.moveRel(x, y)
            
            if self.cursor_in_parameters_field():
                self.button_updown(msg.left_y/100)
                self.button_leftright(msg.left_x/200)
                        
            # !!!
            if self.cursor_in_wplist():
                self.update_scroll_arrows(1)
                self.wp_listbox.yview_scroll(int(msg.left_y/100), "units")

            if msg.button != self.button:
                self.button = msg.button
                if msg.button == 100:
                    pyautogui.click()

    # Read switch SWA status
    def cb_swa(self, msg):
        if msg.data != self.swa:
            self.swa = msg.data
            if msg.data == SW_DOWN: # 3 = switch down to activate function
                if self.swb == SW_DOWN: 
                    pyautogui.click()

    # Read switch SWA status
    def cb_swb(self, msg):
        self.swb = msg.data

    # Read WP list from the FCU
    def cb_mission_wps(self, data): 
        if data != self.wps:
            self.wps = data
            self.current = data.current_seq
            rospy.loginfo("self.wps: {0}".format(len(self.wps.waypoints)))
            rospy.loginfo("wplist len, {0}".format(len(self.wplist)))
            self.wplist.clear()
            for wp in self.wps.waypoints[1:]:
                rospy.loginfo("FCU waypoint {0}, {1}".format(wp.x_lat, wp.y_long))
                listwp = ListWP(wp.x_lat, wp.y_long)
                rospy.loginfo("listwp {0}, {1}".format(listwp.x_lat, listwp.y_long))
                self.wplist.append(listwp)
            rospy.loginfo("wplist len, {0}".format(len(self.wplist)))
            self.set_local_coordinates()
            
            # Update listbox
            self.wp_listbox.delete(0, tk.END)
            for i in range(len(self.wplist)):
                rospy.loginfo("Inserting {0} / {1}: {2}, {3}".format(i, len(self.wplist), round(self.wplist[i].x_m,1), round(self.wplist[i].y_m,1)))
                #if i == 0: self.wp_listbox.insert(tk.END, self.wp_listbox_0_text[self.langindex])
                self.wp_listbox.insert(tk.END, str(i+1) + " (" + str(round(self.wplist[i].x_m,1)) + ", " + str(round(self.wplist[i].y_m,1)) + ")")
                    
            #Update label_cur_tot
            self.label_cur_tot.config(text=self.label_cur_tot_text[self.langindex] + str(self.current) + "/" + str(len(self.wplist)))
            
            #Update map
            self.draw_map()

    # Handle WP list
    def set_local_coordinates(self):
        if len(self.wplist) > 0:
            min_x_lat = self.wplist[0].x_lat
            min_y_long = self.wplist[0].y_long
            max_x_lat = self.wplist[0].x_lat
            max_y_long = self.wplist[0].y_long

            for wp in self.wplist[1:]:
                min_x_lat = min(min_x_lat, wp.x_lat)
                min_y_long = min(min_y_long, wp.y_long)
                max_x_lat = max(max_x_lat, wp.x_lat)
                max_y_long = max(max_y_long, wp.y_long)

            for wp in self.wplist:
                wp.y_m = (wp.x_lat - min_x_lat) * self.k_lat
                wp.x_m = (wp.y_long - min_y_long) * self.k_long
            
            loc_y_range = (max_x_lat - min_x_lat) * self.k_lat
            loc_x_range = (max_y_long - min_y_long) * self.k_long
            self.total_range = max(loc_y_range, loc_x_range)
            rospy.loginfo("Total range: {0}".format(self.total_range))

    def draw_map(self):
        self.wp_map.delete("all")
        for wp in self.wplist:
            k = 230 / (max (5.0, self.total_range))
            x1, y1 = (int(wp.x_m * k) + 5), (230 - int(wp.y_m * k) + 5)
            x2, y2 = (int(wp.x_m * k) + 8), (230 - int(wp.y_m * k) + 8)
            #rospy.loginfo("dot: {0}, {1}, {2}".format(x1, y1, wp.selected))
            if wp.selected:
                self.wp_map.create_oval(x1-2, y1-2, x2+2, y2+2, fill='gold')
            else:
                self.wp_map.create_oval(x1, y1, x2, y2, fill='white')
            
    # Read GPS position data
    def cb_global_position(self, data): 
        self.global_position = data
        self.got_gp = True
        self.label_xlat.config(text=str(round(self.global_position.latitude,6)) + " N")
        self.label_ylong.config(text=str(round(self.global_position.longitude,6)) + " E")

        
    def cb_wpinfo(self, data):
        self.wpinfo_data = data
        
    def run(self):
        rate = rospy.Rate(50)
        
        root.title("Bunker autopilot GUI")
                        
        #frame_arm = ServiceFrame(root, bg="green")
        #button_arm = ServiceButton(frame_arm, text="", width=4, command=lambda: self.service_arming(1))
        #button_arm.pack()
        #frame_mod = ServiceFrame(root, bg="red")
        #button_mod = ServiceButton(frame_mod, text="A", width=4, command=lambda: self.service_mode(2))
        #button_mod.pack()
        
        self.f_buttons.grid(row=1, rowspan=2, column=3, padx=15, pady=(0,15))  # ROOT
        self.label_arm.grid(row=0, column=1)
        self.label_mod.grid(row=0, column=2)
        
        #frame_arm.grid(row=0, column=7, sticky="E")
        #frame_mod.grid(row=1, column=7, sticky="E")

        self.frame_add.grid(row=3, column=6, columnspan=2, rowspan=2)
        self.frame_rem.grid(row=5, column=6, columnspan=2, rowspan=2)
        self.frame_clr.grid(row=7, column=6, columnspan=2, rowspan=2)
        self.frame_bkw.grid(row=9, column=6, columnspan=2, rowspan=2)
        self.frame_off.grid(row=11, column=6, columnspan=2, rowspan=2)
        self.frame_rot.grid(row=13, column=6, columnspan=2)
        self.frame_mir.grid(row=14, column=6, columnspan=2)
        
        root.protocol("WM_DELETE_WINDOW", root.destroy)
        root.mainloop()

if __name__ == "__main__":
    root = tk.Tk()
    rospy.init_node("autopilot_gui")
    pyautogui.PAUSE = 0
    k = Ohjelma()
    k.run()
