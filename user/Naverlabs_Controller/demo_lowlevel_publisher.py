# Wang Yinuo, 07/25/2021, dbdxwyn@163.com
# If you want to control robot by python, use this example and add your controller to it.
import lcm
from exlcm import lowlevel_cmd, lowlevel_state

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
cmd.tau_ff = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
cmd.f_ff = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)

# LCM communication
lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=255")

try:
    while True:
        lc.publish("low_level_cmds", cmd.encode())
except KeyboardInterrupt:
    pass
