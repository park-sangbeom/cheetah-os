# Wang Yinuo, 07/25/2021, dbdxwyn@163.com
# If you want to control robot by python, use this example and add your controller to it.

import lcm
from lcm_msg import lowlevel_cmd, lowlevel_state
import time

# Receive robot state from C++
def robot_state_handler(channel, data):
    msg = lowlevel_state.decode(data)
    print("Received message on channel \"%s\"" % channel)
    print("   q    = %s" % str(msg.q))
    print("   qd   = %s" % str(msg.qd))
    print("   p    = %s" % str(msg.p))
    print("   v    = %s" % str(msg.v))
    print("   rpy         = %s" % str(msg.rpy))
    print("   position    = %s" % str(msg.position))
    print("   orientation = %s" % str(msg.quat))
    print("   vWorld      = %s" % str(msg.vWorld))
    print("   vBody       = %s" % str(msg.vBody))
    print("   omegaWorld  = %s" % str(msg.omegaWorld))
    print("   omegaBody   = %s" % str(msg.omegaBody))
    print("   aWorld      = %s" % str(msg.aWorld))
    print("   aBody       = %s" % str(msg.aBody))
    print(" ")


# Prepare commands for LCM
cmd = lowlevel_cmd()
cmd.q_des = (0.1,-0.8, 1.62, -0.1,-0.8, 1.62,0.1,-0.8, 1.62, -0.1,-0.8, 1.62)
cmd.qd_des = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
cmd.p_des = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
cmd.v_des = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
cmd.kp_joint = (300, 300, 300, 300, 300, 300, 300, 300, 300, 300, 300, 300)
cmd.kd_joint = (15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15)
cmd.kp_cartesian = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
cmd.kd_cartesian = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
cmd.tau_ff = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)#(0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5)
cmd.f_ff = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)

# LCM communication
lc = lcm.LCM()
lc.subscribe("low_level_states", robot_state_handler)
s = time.time()
cnt = 0 
try:
    while True:
        cnt+=1
        if time.time()-s >1:
            print("time: ", time.time()-s, cnt)
        if cnt==1500:
            break 
        lc.handle()
        lc.publish("low_level_cmds", cmd.encode())
except KeyboardInterrupt:
    pass

# # LCM communication
# lc = lcm.LCM()
# lc.subscribe("low_level_states", robot_state_handler)
# times = 10000
# try:
#     for i in range(times):
#         q_des = [-0.5346+(0.5346+0.1*i/times),-1.099+(-1.099+0.8)*i/times, 2.6585-(2.6585-1.62)*i/times, \
#                 -0.5346+(0.5346+0.1*i/times),-1.099+(-1.099+0.8)*i/times, 2.6585-(2.6585-1.62)*i/times,\
#                 -0.5346+(0.5346+0.1*i/times),-1.099+(-1.099+0.8)*i/times, 2.6585-(2.6585-1.62)*i/times, \
#                 -0.5346+(0.5346+0.1*i/times),-1.099+(-1.099+0.8)*i/times, 2.6585-(2.6585-1.62)*i/times]
#         cmd.q_des = q_des #(0.1,-0.8, 1.62, -0.1,-0.8, 1.62,0.1,-0.8, 1.62, -0.1,-0.8, 1.62)
#         cmd.qd_des = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
#         cmd.p_des = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
#         cmd.v_des = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
#         cmd.kp_joint = (15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15)#(300, 300, 300, 300, 300, 300, 300, 300, 300, 300, 300, 300)
#         cmd.kd_joint = (15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15)
#         cmd.kp_cartesian = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
#         cmd.kd_cartesian = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
#         cmd.tau_ff = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
#         cmd.f_ff = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
#         lc.handle()
#         lc.publish("low_level_cmds", cmd.encode())
# except KeyboardInterrupt:
#     pass