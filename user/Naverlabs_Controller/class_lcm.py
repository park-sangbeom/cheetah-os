import lcm
from exlcm import lowlevel_cmd, lowlevel_state
import json 
import numpy as np 
from threading import Timer
from time import sleep 
import time 
import os 

class RepeatedTimer(object):
    def __init__(self, interval, function, *args, **kwargs):
        self._timer     = None
        self.interval   = interval
        self.function   = function
        self.args       = args
        self.kwargs     = kwargs
        self.is_running = False
        self.start()

    def _run(self):
        self.is_running = False
        self.start()
        self.function(*self.args, **self.kwargs)

    def start(self):
        if not self.is_running:
            self._timer = Timer(self.interval, self._run)
            self._timer.start()
            self.is_running = True

    def stop(self):
        self._timer.cancel()
        self.is_running = False

class LightweightCommunicationsMarshalling:
    def __init__(self, 
                 sub_channel="low_level_states",
                 pub_channel="low_level_cmds",
                 udpm="udpm://239.255.76.67:7667?ttl=255",
                 txt_file="data.txt",
                 VERBOSE=True):
        self.lc          = lcm.LCM(udpm)
        self.sub_channel = sub_channel 
        self.pub_channel = pub_channel 
        self.txt_file    = txt_file 
        self.VERBOSE     = VERBOSE
        self.msg         = None 
        self.lowlevel_cmd = lowlevel_cmd()
        self.lowlevel_state = lowlevel_state()
        if not os.path.exists('./'+self.txt_file):
            open('./'+self.txt_file,'w')
        self.lc.subscribe(self.sub_channel, self.robot_state_handler)

    def robot_state_handler(self, channel, data):
        msg = self.lowlevel_state.decode(data)
        lcm.EventLog(self.txt_file, mode='w',overwrite=True).write_event(1, channel,data)

    def subscriber(self):
        self.lc.handle()
        log = lcm.EventLog(self.txt_file, "r")
        for event in log:
            if event.channel == self.sub_channel:
                self.msg = self.lowlevel_state.decode(event.data)
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


