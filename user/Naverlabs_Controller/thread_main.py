import lcm
from exlcm import lowlevel_cmd, lowlevel_state
import json 
import numpy as np 
from threading import Timer
from time import sleep 
import time 
from class_lcm import LightweightCommunicationsMarshalling, RepeatedTimer

if __name__=="__main__":
    lcm_module = LightweightCommunicationsMarshalling(udpm="udpm://239.255.76.67:7667?ttl=255",
                                               sub_channel="low_level_states", 
                                               pub_channel="low_level_cmds")
    # Get init state 
    lcm_module.subscriber()

    # Initialize 
    HZ = 100 
    TIME = 1
    INITSTEPS = HZ*TIME
    frequency = TIME/HZ
    for step in range(INITSTEPS):
        q_des = [lcm_module.msg.q[0]+(0.1-lcm_module.msg.q[0])*(step/INITSTEPS), lcm_module.msg.q[1]+(-0.8-lcm_module.msg.q[1])*(step/INITSTEPS), lcm_module.msg.q[2]+(1.62-lcm_module.msg.q[2])*(step/INITSTEPS),
                lcm_module.msg.q[3]+(0.1-lcm_module.msg.q[3])*(step/INITSTEPS), lcm_module.msg.q[4]+(-0.8-lcm_module.msg.q[4])*(step/INITSTEPS), lcm_module.msg.q[5]+(1.62-lcm_module.msg.q[5])*(step/INITSTEPS),
                lcm_module.msg.q[6]+(0.1-lcm_module.msg.q[6])*(step/INITSTEPS), lcm_module.msg.q[7]+(-0.8-lcm_module.msg.q[7])*(step/INITSTEPS), lcm_module.msg.q[8]+(1.62-lcm_module.msg.q[8])*(step/INITSTEPS),
                lcm_module.msg.q[9]+(0.1-lcm_module.msg.q[9])*(step/INITSTEPS), lcm_module.msg.q[10]+(-0.8-lcm_module.msg.q[10])*(step/INITSTEPS), lcm_module.msg.q[11]+(1.62-lcm_module.msg.q[11])*(step/INITSTEPS)]
        rt = RepeatedTimer(frequency, lcm_module.publisher,q_des=q_des,
                        # kp_joint=(17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17),
                        # kd_joint=(0.4,0.4, 0.4,0.4,0.4, 0.4, 0.4,0.4, 0.4, 0.4,0.4, 0.4),
                        kp_joint=(80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80),
                        kd_joint=(1,1, 1,1,1,1, 1,1, 1,1, 1,1,),
                        # kp_cartesian=(50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50),
                        # kd_cartesian=(2.5,2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5,2.5, 2.5, 2.5))
                        kp_cartesian=(500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500),
                        kd_cartesian=(8, 8, 8, 8, 8, 8, 8, 8, 8,8, 8, 8))
        try:
            sleep(frequency)
        finally:
            rt.stop() 
    stand_q = q_des
    # Standing 
    for i in range(1000):
        print(i)
        rt = RepeatedTimer(frequency, lcm_module.publisher,q_des=q_des,
                        # kp_joint=(17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17),
                        # kd_joint=(0.4,0.4, 0.4,0.4,0.4, 0.4, 0.4,0.4, 0.4, 0.4,0.4, 0.4),
                        kp_joint=(80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80),
                        kd_joint=(1,1, 1,1,1,1, 1,1, 1,1, 1,1,),
                        # kp_cartesian=(50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50),
                        # kd_cartesian=(2.5,2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5,2.5, 2.5, 2.5))
                        kp_cartesian=(500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500),
                        kd_cartesian=(8, 8, 8, 8, 8, 8, 8, 8, 8,8, 8, 8))
        lcm_module.subscriber()
        try:
            sleep(frequency)
        finally:
            rt.stop() 
    print("Done,")
    # Down 
    HZ = 1000 
    TIME = 1
    INITSTEPS = HZ*TIME
    frequency = TIME/HZ
    for step in range(INITSTEPS):
        q_des = [stand_q[0]+(stand_q[0]-0)*(step/INITSTEPS), stand_q[1]+(-0.5+stand_q[1])*(step/INITSTEPS), stand_q[2]+(stand_q[2]-1)*(step/INITSTEPS),
                stand_q[3]+(stand_q[3]-0)*(step/INITSTEPS), stand_q[4]+(-0.5+stand_q[4])*(step/INITSTEPS), stand_q[5]+(stand_q[5]-1)*(step/INITSTEPS),
                stand_q[6]+(stand_q[6]-0)*(step/INITSTEPS), stand_q[7]+(-0.5+stand_q[7])*(step/INITSTEPS), stand_q[8]+(stand_q[8]-1)*(step/INITSTEPS),
                stand_q[9]+(stand_q[9]-0)*(step/INITSTEPS), stand_q[10]+(-0.5+stand_q[10])*(step/INITSTEPS), stand_q[11]+(stand_q[11]-1)*(step/INITSTEPS)]
        rt = RepeatedTimer(frequency, lcm_module.publisher,q_des=q_des,
                        # kp_joint=(17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17),
                        # kd_joint=(0.4,0.4, 0.4,0.4,0.4, 0.4, 0.4,0.4, 0.4, 0.4,0.4, 0.4),
                        kp_joint=(80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80),
                        kd_joint=(1,1, 1,1,1,1, 1,1, 1,1, 1,1,),
                        # kp_cartesian=(50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50),
                        # kd_cartesian=(2.5,2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5,2.5, 2.5, 2.5))
                        kp_cartesian=(500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500),
                        kd_cartesian=(8, 8, 8, 8, 8, 8, 8, 8, 8,8, 8, 8))
        try:
            sleep(frequency)
        finally:
            rt.stop() 

    final_q = q_des
    # Standing 
    for i in range(100):
        rt = RepeatedTimer(frequency, lcm_module.publisher,q_des=final_q,
                        # kp_joint=(17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17),
                        # kd_joint=(0.4,0.4, 0.4,0.4,0.4, 0.4, 0.4,0.4, 0.4, 0.4,0.4, 0.4),
                        kp_joint=(80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80),
                        kd_joint=(1,1, 1,1,1,1, 1,1, 1,1, 1,1,),
                        # kp_cartesian=(50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50),
                        # kd_cartesian=(2.5,2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5,2.5, 2.5, 2.5))
                        kp_cartesian=(500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500),
                        kd_cartesian=(8, 8, 8, 8, 8, 8, 8, 8, 8,8, 8, 8))
        lcm_module.subscriber()
        try:
            sleep(frequency)
        finally:
            rt.stop() 