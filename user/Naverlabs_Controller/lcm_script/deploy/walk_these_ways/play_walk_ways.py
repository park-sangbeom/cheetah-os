import isaacgym
assert isaacgym

import sys 
sys.path.append('../')
from go1_gym.envs import *
from go1_gym.envs.base.legged_robot_config import Cfg
from go1_gym.envs.go1.go1_config import config_go1
from go1_gym.envs.go1.velocity_tracking import VelocityTrackingEasyEnv
import torch
import time 
from multiprocessing.connection import Client, Listener
import numpy as np
import pickle as pkl
import glob
import os
import json 
from copy import deepcopy
import lcm 
from lcm_agent import LCMAgent
from wrapper import Wrapper 
from state_estimator import StateEstimator
from stand_up_down import stand_up_publisher
from lowlevel_cmd_publisher import LowLevelCommandPublisher

def load_policy(logdir):
    body = torch.jit.load(logdir + '/checkpoints/body_latest.jit')
    import os
    adaptation_module = torch.jit.load(logdir + '/checkpoints/adaptation_module_latest.jit')

    def policy(obs, info={}):
        i = 0
        latent = adaptation_module.forward(obs["obs_history"].to('cpu'))
        action = body.forward(torch.cat((obs["obs_history"].to('cpu'), latent), dim=-1))
        info['latent'] = latent
        return action

    return policy

def main(lc, control_dt, commands, obs_dim, history_leng, logdir, device='cpu'):
    # Trot gait 
    trot_motion=[]; trot_joint_cnt = 0
    for line in open('trot_joint.json','r'):
        data = json.loads(line)
        trot_motion.append([data['q'][0]+data['q'][1]+data['q'][2]+data['q'][3]])
    kp_joint = [20.0] * 12
    kd_joint = [0.5] * 12

    gaits = {"pronking": [0, 0, 0],
             "trotting": [0.5, 0, 0],
             "bounding": [0, 0.5, 0],
             "pacing": [0, 0, 0.5]}


    policy = load_policy(logdir=logdir)
    stand_up_publisher()
    se = StateEstimator(lc=lc)
    lcm_agent = LCMAgent(se=se, control_dt=control_dt, commands=commands)
    lcm_publisher = LowLevelCommandPublisher()
    se.spin()
    wrapper = Wrapper(env=lcm_agent, obs_history_length=obs_dim*history_leng) 
    obs_dict = wrapper.reset()
    real_time = time.time()
    # 8 seconds for rapid locomotion
    while (time.time()-real_time)<2.0:
        actions = policy(obs_dict)
        obs_dict = wrapper.step(actions)

    # 500 Hz joint trajectory
    while trot_joint_cnt<len(trot_motion[:400]):
        s = time.time()
        lcm_publisher.publisher(q_des=trot_motion[trot_joint_cnt][0],
                    kp_joint=kp_joint,
                    kd_joint=kd_joint)
        trot_joint_cnt +=1 
        time.sleep(max(0.002-(time.time() - s), 0))



if __name__=="__main__":
    label = "gait-conditioned-agility/pretrain-v0/train"
    dirs = glob.glob(f"../runs/{label}/*")
    logdir = sorted(dirs)[0]

    lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=255")
    HZ = 50
    control_dt = 1/HZ
    obs_dim = 70
    history_leng = 30
    gaits = {"pronking": [0, 0, 0],
             "trotting": [0.5, 0, 0],
             "bounding": [0, 0.5, 0],
             "pacing": [0, 0, 0.5]}
    x_vel_cmd, y_vel_cmd, yaw_vel_cmd = 1.5, 0.0, 0.0
    body_height_cmd = 0.0
    step_frequency_cmd = 3.0
    gait = torch.tensor(gaits["trotting"])
    footswing_height_cmd = 0.08
    pitch_cmd = 0.0
    roll_cmd = 0.0
    stance_width_cmd = 0.25
    commands= np.zeros((1,13)) #np.array([0.9, 0, 0.5]) #np.array([2.0, 0, 0.]) #np.array([0.5, 0, 0.9])

    commands[:, 0] = x_vel_cmd
    commands[:, 1] = y_vel_cmd
    commands[:, 2] = yaw_vel_cmd
    commands[:, 3] = body_height_cmd
    commands[:, 4] = step_frequency_cmd
    commands[:, 5:8] = gait
    commands[:, 8] = 0.5
    commands[:, 9] = footswing_height_cmd
    commands[:, 10] = pitch_cmd
    commands[:, 11] = roll_cmd
    commands[:, 12] = stance_width_cmd

    main(lc, control_dt, commands, obs_dim, history_leng, logdir=logdir)
