import lcm
import sys
sys.path.append('..')
from lowlevel_msg.lowlevel_cmd import lowlevel_cmd 
import numpy as np 
from threading import Timer
from time import sleep 
import time 
import os 

class LowLevelCommandPublisher:
    def __init__(self,                  
                 pub_channel="low_level_cmds",
                 udpm="udpm://239.255.76.67:7667?ttl=255"):
        self.lc          = lcm.LCM(udpm)
        self.pub_channel = pub_channel
        self.lowlevel_cmd = lowlevel_cmd()

    def publisher(self, q_des=(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0), 
                        q_vel=(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0), 
                        kp_joint=(20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20),
                        kd_joint=(5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5),
                        kp_cartesian=(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
                        kd_cartesian=(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)):
        self.lowlevel_cmd.q_des = q_des
        self.lowlevel_cmd.qd_des = q_vel
        self.lowlevel_cmd.p_des = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        self.lowlevel_cmd.v_des = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        self.lowlevel_cmd.kp_joint = kp_joint
        self.lowlevel_cmd.kd_joint = kd_joint
        self.lowlevel_cmd.kp_cartesian = kp_cartesian
        self.lowlevel_cmd.kd_cartesian = kd_cartesian
        self.lowlevel_cmd.tau_ff = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        self.lowlevel_cmd.f_ff = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        self.lc.publish(self.pub_channel, self.lowlevel_cmd.encode())

class LowLevelStateSubscriber:
    def __init__(self,                  
                 sub_channel="low_level_states",
                 udpm="udpm://239.255.76.67:7667?ttl=255",
                 txt_file="data.txt",
                 VERBOSE=False):
        self.lc          = lcm.LCM(udpm)
        self.sub_channel = sub_channel 
        self.txt_file    = txt_file 
        self.VERBOSE     = VERBOSE
        self.msg         = None 
        self.lowlevel_state = lowlevel_state()
        self.lc.subscribe(self.sub_channel, self.robot_state_handler)

    def robot_state_handler(self, channel, data):
        self.msg = self.lowlevel_state.decode(data)
        if self.VERBOSE:
            print("   q    = %s" % str(self.msg.q))
            print("   qd   = %s" % str(self.msg.qd))
            print("   p    = %s" % str(self.msg.p))
            print("   v    = %s" % str(self.msg.v))
            print("   rpy         = %s" % str(self.msg.rpy))
            print("   position    = %s" % str(self.msg.position))
            print("   orientation = %s" % str(self.msg.quat))
            print("   vWorld      = %s" % str(self.msg.vWorld))
            print("   vBody       = %s" % str(self.msg.vBody))
            print("   omegaWorld  = %s" % str(self.msg.omegaWorld))
            print("   omegaBody   = %s" % str(self.msg.omegaBody))
            print("   aWorld      = %s" % str(self.msg.aWorld))
            print("   aBody       = %s" % str(self.msg.aBody))

class LowLevelCommandSubscriber:
    def __init__(self,                  
                 sub_channel="low_level_cmds",
                 udpm="udpm://239.255.76.67:7667?ttl=255",
                 txt_file="data.txt",
                 VERBOSE=False):
        self.lc          = lcm.LCM(udpm)
        self.sub_channel = sub_channel 
        self.txt_file    = txt_file 
        self.VERBOSE     = VERBOSE
        self.msg         = None 
        self.lowlevel_cmd = lowlevel_cmd()
        self.lc.subscribe(self.sub_channel, self.robot_state_handler)

    def robot_state_handler(self, channel, data):
        self.msg = self.lowlevel_cmd.decode(data)
        if self.VERBOSE:
            print("   q_des    = %s" % str(self.msg.q_des))
            print("   qd_des   = %s" % str(self.msg.qd_des))
            print("   p_des    = %s" % str(self.msg.p_des))
            print("   v_des    = %s" % str(self.msg.v_des))
            print("   kp_joint         = %s" % str(self.msg.kp_joint))
            print("   kd_joint    = %s" % str(self.msg.kd_joint))
            print("   kp_cartesian = %s" % str(self.msg.kp_cartesian))
            print("   kd_cartesian      = %s" % str(self.msg.kd_cartesian))
            print("   tau_ff       = %s" % str(self.msg.tau_ff))
            print("   f_ff  = %s" % str(self.msg.f_ff))
