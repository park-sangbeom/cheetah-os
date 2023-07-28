# Wang Yinuo, 07/25/2021, dbdxwyn@163.com
# If you want to control robot by python, use this example and add your controller to it.
import lcm
from exlcm import lowlevel_cmd, lowlevel_state
import json 
import numpy as np 
import time 

# Prepare commands for LCM
cmd = lowlevel_cmd()

# LCM communication
lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=255")
times = 1000
interval = 0.1
for i in range(times):
    s = time.time()
    q_des = [-0.5346+(0.5346+0.1*i/times),-1.099+(-1.099+0.8)*i/times, 2.6585-(2.6585-1.62)*i/times, \
            -0.5346+(0.5346+0.1*i/times),-1.099+(-1.099+0.8)*i/times, 2.6585-(2.6585-1.62)*i/times,\
            -0.5346+(0.5346+0.1*i/times),-1.099+(-1.099+0.8)*i/times, 2.6585-(2.6585-1.62)*i/times, \
            -0.5346+(0.5346+0.1*i/times),-1.099+(-1.099+0.8)*i/times, 2.6585-(2.6585-1.62)*i/times]
    cmd.q_des = q_des
    cmd.qd_des = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
    cmd.p_des = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
    cmd.v_des =(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
    cmd.kp_joint = (20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20)
    cmd.kd_joint = (0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5)
    cmd.kp_cartesian = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
    cmd.kd_cartesian = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
    cmd.tau_ff = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
    cmd.f_ff = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
    lc.publish("low_level_cmds", cmd.encode())
    time.sleep(max(interval-(time.time() - s), 0))
