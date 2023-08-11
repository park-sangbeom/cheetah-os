import lcm
import sys
sys.path.append('..')
from msg.lowlevel_msg.lowlevel_cmd import lowlevel_cmd 
from msg.lowlevel_msg.lowlevel_state import lowlevel_state
from msg.spi_msg.spi_command_t import spi_command_t
from msg.spi_msg.spi_data_t import spi_data_t
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

class SPIDataSubscriber:
    def __init__(self,                  
                 sub_channel="spi_data",
                 udpm="udpm://239.255.76.67:7667?ttl=255",
                 txt_file="data.txt",
                 VERBOSE=False):
        self.lc          = lcm.LCM(udpm)
        self.sub_channel = sub_channel 
        self.txt_file    = txt_file 
        self.VERBOSE     = VERBOSE
        self.msg         = None 
        self.spi_data_t = spi_data_t()
        self.lc.subscribe(self.sub_channel, self.robot_state_handler)

    def robot_state_handler(self, channel, data):
        self.msg = self.spi_data_t.decode(data)
        if self.VERBOSE:
            print("   q_abad    = %s" % str(self.msg.q_abad))
            print("   q_hip   = %s" % str(self.msg.q_hip))
            print("   q_knee    = %s" % str(self.msg.q_knee))
            print("   qd_abad    = %s" % str(self.msg.qd_abad))
            print("   qd_hip         = %s" % str(self.msg.qd_hip))
            print("   qd_knee    = %s" % str(self.msg.qd_knee))
            # print("   flags = %s" % str(self.msg.kp_cartflagsesian))
            print("   spi_driver_status      = %s" % str(self.msg.spi_driver_status))

class SPICommandSubscriber:
    def __init__(self,                  
                 sub_channel="spi_command",
                 udpm="udpm://239.255.76.67:7667?ttl=255",
                 txt_file="data.txt",
                 VERBOSE=False):
        self.lc          = lcm.LCM(udpm)
        self.sub_channel = sub_channel 
        self.txt_file    = txt_file 
        self.VERBOSE     = VERBOSE
        self.msg         = None 
        self.spi_command_t = spi_command_t()
        self.lc.subscribe(self.sub_channel, self.robot_state_handler)

    def robot_state_handler(self, channel, data):
        self.msg = self.spi_command_t.decode(data)
        if self.VERBOSE:
            print("   q_des_abad    = %s" % str(self.msg.q_des_abad))
            print("   q_des_hip   = %s" % str(self.msg.q_des_hip))
            print("   q_des_knee    = %s" % str(self.msg.q_des_knee))
            print("   qd_des_abad    = %s" % str(self.msg.qd_des_abad))
            print("   qd_des_hip         = %s" % str(self.msg.qd_des_hip))
            print("   qd_des_knee    = %s" % str(self.msg.qd_des_knee))
            print("   kp_abad = %s" % str(self.msg.kp_abad))
            print("   kp_hip      = %s" % str(self.msg.kp_hip))
            print("   kp_knee   = %s" % str(self.msg.kp_knee))
            print("   kd_abad    = %s" % str(self.msg.kd_abad))
            print("   kd_hip    = %s" % str(self.msg.kd_hip))
            print("   kd_knee         = %s" % str(self.msg.kd_knee))
            print("   tau_abad_ff    = %s" % str(self.msg.tau_abad_ff))
            print("   tau_hip_ff = %s" % str(self.msg.tau_hip_ff))
            print("   tau_knee_ff      = %s" % str(self.msg.tau_knee_ff))
            print("   flags      = %s" % str(self.msg.flags))


if __name__=="__main__":
    lcm_pub = LowLevelCommandPublisher()
    lcm_sub = SPIDataSubscriber(VERBOSE=True)
    interval = 0.002 
    s = time.time()
    print("Start")
    while True: 
        s = time.time()
        lcm_sub.lc.handle()
        # lcm_pub.publisher(
        # q_des=(-0.807, -1., 2., \
        #         0.807, -1.2, 2.4, \
        #         -0.807, -1.2, 2.4, \
        #         0.807, -1.2, 2.4, ), 
        # q_vel=(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0), 
        # kp_joint=(20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20),
        # kd_joint=(0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5),
        # kp_cartesian=(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
        # kd_cartesian=(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
        time.sleep(max(interval-(time.time() - s), 0))