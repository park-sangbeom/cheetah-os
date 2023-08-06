import isaacgym
assert isaacgym
import sys 
sys.path.append('../')
from isaacgym.torch_utils import *
from mini_gym.envs import *
from mini_gym.envs.base.legged_robot_config import Cfg
from mini_gym.envs.mini_cheetah.mini_cheetah_config import config_mini_cheetah
from ml_logger import logger
from pathlib import Path
from mini_gym import MINI_GYM_ROOT_DIR
from mini_gym_learn.ppo.ppo import PPO_Args
from mini_gym_learn.ppo.actor_critic import AC_Args
from mini_gym_learn.ppo import RunnerArgs
from ml_logger import logger
from mini_gym_learn.ppo.actor_critic import ActorCritic
import torch
import time 
from multiprocessing.connection import Client, Listener
import numpy as np
import glob
import os
from copy import deepcopy
import lcm 
from lcm_agent import LCMAgent
from wrapper import Wrapper 
from state_estimator import StateEstimator
from stand_up_down import stand_up_publisher

def load_policy(device='cpu'):
    # prepare environment
    config_mini_cheetah(Cfg)
    params = logger.load_pkl('parameters.pkl')

    if 'kwargs' in params[0]:
        deps = params[0]['kwargs']
        AC_Args._update(deps)
        PPO_Args._update(deps)
        RunnerArgs._update(deps)
        Cfg.terrain._update(deps)
        Cfg.commands._update(deps)
        Cfg.normalization._update(deps)
        Cfg.env._update(deps)
        Cfg.domain_rand._update(deps)
        Cfg.rewards._update(deps)
        Cfg.reward_scales._update(deps)
        Cfg.perception._update(deps)
        Cfg.domain_rand._update(deps)
        Cfg.control._update(deps)

    # turn off DR for evaluation script
    Cfg.domain_rand.push_robots = False
    Cfg.domain_rand.randomize_friction = False
    Cfg.domain_rand.randomize_gravity = False
    Cfg.domain_rand.randomize_restitution = False
    Cfg.domain_rand.randomize_motor_offset = False
    Cfg.domain_rand.randomize_motor_strength = False
    Cfg.domain_rand.randomize_friction_indep = False
    Cfg.domain_rand.randomize_ground_friction = False
    Cfg.domain_rand.randomize_base_mass = False
    Cfg.domain_rand.randomize_Kd_factor = False
    Cfg.domain_rand.randomize_Kp_factor = False
    Cfg.domain_rand.randomize_joint_friction = False
    Cfg.domain_rand.randomize_com_displacement = False

    Cfg.env.num_recording_envs = 1
    Cfg.env.num_envs = 1
    Cfg.terrain.num_rows = 3
    Cfg.terrain.num_cols = 5
    Cfg.terrain.border_size = 0

    actor_critic = ActorCritic(
        num_obs=Cfg.env.num_observations,
        num_privileged_obs=Cfg.env.num_privileged_obs,
        num_obs_history=Cfg.env.num_observations * \
                        Cfg.env.num_observation_history,
        num_actions=Cfg.env.num_actions)

    print(logger.prefix)
    print(logger.glob("*"))
    weights = logger.load_torch("checkpoints/ac_weights_last.pt")
    actor_critic.load_state_dict(state_dict=weights)
    actor_critic.to(device)
    policy = actor_critic.act_inference
    return policy

def main(lc, control_dt, commands, obs_dim, history_leng, device='cpu'):
    policy = load_policy(device=device)
    stand_up_publisher()
    se = StateEstimator(lc=lc)
    lcm_agent = LCMAgent(se=se, control_dt=control_dt, commands=commands)
    se.spin()
    wrapper = Wrapper(env=lcm_agent, obs_history_length=obs_dim*history_leng) 
    obs_dict = wrapper.reset()
    while True:
        actions = policy(obs_dict)
        obs_dict = wrapper.step(actions)

if __name__=="__main__":
    recent_runs = sorted(glob.glob(f"{MINI_GYM_ROOT_DIR}/runs/rapid-locomotion/*/*/*"), key=os.path.getmtime)
    logger.configure(Path(recent_runs[-1]).resolve())
    lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=255")
    HZ = 50
    control_dt = 1/HZ
    obs_dim = 42 
    history_leng = 15
    commands= np.array([2.0, 0, 0.]) #np.array([0.5, 0, 0.9])
    main(lc, control_dt, commands, obs_dim, history_leng)
