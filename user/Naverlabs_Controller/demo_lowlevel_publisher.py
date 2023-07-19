# Wang Yinuo, 07/25/2021, dbdxwyn@163.com
# If you want to control robot by python, use this example and add your controller to it.
import lcm
from exlcm import lowlevel_cmd, lowlevel_state
import json 
import numpy as np 
walking_data = [json.loads(line) for line in open('./test.json', 'r')]
walking_np = np.array(walking_data)
print(walking_np.shape)
# Prepare commands for LCM
cmd = lowlevel_cmd()


def interpolation(x_anchor, num_interpol, key):
    for anchor_idx in range(len(x_anchor)):
        if (anchor_idx+1) == len(x_anchor):
            break
        if anchor_idx ==0:
            interpoled_x = np.linspace(x_anchor[anchor_idx][key], x_anchor[anchor_idx+1][key], num_interpol)
            interpoled_x_arr = interpoled_x
        else:
            interpoled_x = np.linspace(x_anchor[anchor_idx][key], x_anchor[anchor_idx+1][key], num_interpol)
            interpoled_x_arr = np.append(interpoled_x_arr, interpoled_x, axis=0)
    return interpoled_x_arr

interpoled_dof_pos = interpolation(walking_np, 500, 'dof_pos')
interpoled_dof_vel = interpolation(walking_np, 500, 'dof_vel')
# cmd.q_des = (0.1,-0.8, 1.62, -0.1,-0.8, 1.62,0.1,-0.8, 1.62, -0.1,-0.8, 1.62)
# cmd.qd_des = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
# cmd.p_des = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
# cmd.v_des = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
# cmd.kp_joint = (300, 300, 300, 300, 300, 300, 300, 300, 300, 300, 300, 300)
# cmd.kd_joint = (15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15)
# cmd.kp_cartesian = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
# cmd.kd_cartesian = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
# cmd.tau_ff = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
# cmd.f_ff = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)

# LCM communication
lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=255")
times = 100
for i in range(times):
        q_des = [-0.5346+(0.5346+0.1*i/times),-1.099+(-1.099+0.8)*i/times, 2.6585-(2.6585-1.62)*i/times, \
                -0.5346+(0.5346+0.1*i/times),-1.099+(-1.099+0.8)*i/times, 2.6585-(2.6585-1.62)*i/times,\
                -0.5346+(0.5346+0.1*i/times),-1.099+(-1.099+0.8)*i/times, 2.6585-(2.6585-1.62)*i/times, \
                -0.5346+(0.5346+0.1*i/times),-1.099+(-1.099+0.8)*i/times, 2.6585-(2.6585-1.62)*i/times]
        cmd.q_des = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        cmd.qd_des = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        cmd.p_des = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        cmd.v_des =(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        cmd.kp_joint = (20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20)#(300, 300, 300, 300, 300, 300, 300, 300, 300, 300, 300, 300)
        cmd.kd_joint = (5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5)#(15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15)
        cmd.kp_cartesian = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        cmd.kd_cartesian = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        cmd.tau_ff = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        cmd.f_ff = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        lc.publish("low_level_cmds", cmd.encode())



while True:
    for idx, (dof_pos, dof_vel) in enumerate(zip(interpoled_dof_pos, interpoled_dof_vel)):
        cmd.q_des = dof_pos
        cmd.qd_des = dof_vel#(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        cmd.p_des = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        cmd.v_des = dof_vel
        cmd.kp_joint = (300, 300, 300, 300, 300, 300, 300, 300, 300, 300, 300, 300) # (20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20)
        cmd.kd_joint = (15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15) # (5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5)
        cmd.kp_cartesian = (5, 0, 0, 5, 0, 0, 5, 0, 0,5, 0, 0)#(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        cmd.kd_cartesian = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        cmd.tau_ff =  (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        cmd.f_ff = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        if idx%20==0:
            lc.publish("low_level_cmds", cmd.encode())

try:
    while True:
        lc.publish("low_level_cmds", cmd.encode())
except KeyboardInterrupt:
    pass
