import lcm 
import numpy as np 
import time 
import torch 
from state_estimator import StateEstimator
from lowlevel_msg.lowlevel_cmd import lowlevel_cmd

def np2torch(x_np,device='cpu'):
    if x_np is None:
        x_torch = None
    else:
        x_torch = torch.tensor(x_np,dtype=torch.float32,device=device)
    return x_torch

class LCMAgent:
    def __init__ (self, se: StateEstimator, control_dt= 0.002, commands=None, device='cpu'):
        self.se = se 
        self.control_dt = control_dt
        self.gravity_vector = np.zeros(3)
        self.dof_pos = np.zeros(12)
        self.dof_vel = np.zeros(12)
        self.body_linear_vel = np.zeros(3)
        self.body_angular_vel = np.zeros(3)

        self.joint_idxs = self.se.joint_idxs
        self.p_gains = [20]*12 
        self.d_gains = [0.5]*12
        self.gait_indices = torch.zeros(1, dtype=torch.float)
        self.clock_inputs = torch.zeros(1, 4, dtype=torch.float)
        self.device = device 

        self.commands = commands.reshape(1,-1)
        self.commands_scale = torch.tensor([[2.0, 2.0, 0.25]], device=self.device, requires_grad=False,) # TODO change this
        self.actions = torch.zeros(12)
        self.joint_pos_target = torch.zeros(12)
        self.action_prev = torch.zeros(12)
        self.lowlevel_cmd = lowlevel_cmd()
        self.default_dof_pos = np.array([0.1, -0.8, 1.62,
                                         -0.1,-0.8, 1.62,
                                          0.1, -0.8, 1.62,
                                          -0.1,-0.8, 1.62])

    def reset(self):
        self.actions = np2torch(self.default_dof_pos, self.device)#torch.zeros(12)
        self.time = time.time()
        self.timestep = 0
        return self.get_obs()

    def get_obs(self):
        self.gravity_vector = np2torch(self.se.get_gravity_vector(), self.device).reshape(1,-1)
        self.dof_pos = np2torch(self.se.get_dof_pos(), self.device).reshape(1,-1)
        self.dof_vel = np2torch(self.se.get_dof_vel(), self.device).reshape(1,-1)
        self.base_lin_vel = np2torch(self.se.get_lin_vel(), self.device).reshape(1,-1)
        self.base_ang_vel = np2torch(self.se.get_ang_vel(), self.device).reshape(1,-1)

        obs = torch.cat((self.base_lin_vel * 2.0,
                        self.base_ang_vel  * 0.25,
                        self.gravity_vector,
                        (np2torch(self.commands[:,:3],self.device) * self.commands_scale),
                        (self.dof_pos.reshape(1,-1) - self.default_dof_pos.reshape(1,-1)) * 1.0,
                        self.dof_vel.reshape(1,-1) * 0.05,
                        self.actions.reshape(1,-1)),dim=-1) 
        return obs
    
    def step(self, actions):
        self.action_prev = self.actions 
        self.actions = actions 
        self.publish_lowlevel_cmd(self.actions)
        time.sleep(max(self.control_dt - (time.time() - self.time), 0))
        if self.timestep % 100 == 0: print(f'frq: {1 / (time.time() - self.time)} Hz');
        self.time = time.time()
        obs = self.get_obs()
        return obs 

    def publish_lowlevel_cmd(self, actions):
        self.joint_pos_target = actions[0,:12].detach().cpu().numpy()*0.25 
        self.joint_pos_target[[0,3,6,9]]*=0.5 
        self.joint_pos_target = self.joint_pos_target 
        self.joint_pos_target += self.default_dof_pos 
        joint_pos_target = self.joint_pos_target[self.joint_idxs]
        joint_vel_target = np.zeros(12)
        self.lowlevel_cmd.q_des = joint_pos_target
        self.lowlevel_cmd.qd_des = joint_vel_target
        self.lowlevel_cmd.p_des = np.zeros(12)
        self.lowlevel_cmd.v_des = np.zeros(12)
        self.lowlevel_cmd.kp_joint = self.p_gains
        self.lowlevel_cmd.kd_joint = self.d_gains
        self.lowlevel_cmd.kp_cartesian = np.zeros(12)
        self.lowlevel_cmd.kd_cartesian = np.zeros(12)
        self.lowlevel_cmd.tau_ff = np.zeros(12)
        self.lowlevel_cmd.f_ff = np.zeros(12)
        self.se.lc.publish("low_level_cmds", self.lowlevel_cmd.encode())

if __name__=="__main__":
    lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=255")
    se = StateEstimator(lc)
    lcm_agent = LCMAgent(se=se, commands=np.array([0.5, 0, 0.9]))
    se.spin()
    obs = lcm_agent.get_obs()  
    print('obs', obs)