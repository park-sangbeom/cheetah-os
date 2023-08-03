
import isaacgym
assert isaacgym
import sys 
sys.path.append('../../')
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

def np2torch(x_np,device='cpu'):
    if x_np is None:
        x_torch = None
    else:
        x_torch = torch.tensor(x_np,dtype=torch.float32,device=device)
    return x_torch


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

def T(qin):
    tmp = list(qin).copy()
    qout = tmp[3:6] + tmp[0:3] + tmp[9:12] + tmp[6:9] 
    assert len(qout) == 12
    return qout


def subscriber_main(policy, subscribe, publish):
    msg = None
    obs_dict = dict()
    init = True
    HZ = 500 
    interval = 1/HZ
    agent_num = 1 
    obs_dim = 42 
    history_leng = 15
    gait_change_time_step = 0
    default_q = [0.1, -0.8, 1.62, \
                -0.1, -0.8, 1.62, \
                0.1, -0.8, 1.62, \
                -0.1, -0.8, 1.62]
    commands = [0.9,0.0,0.5] # Slow Rotate Motion 
    action_prev = np2torch(np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]).reshape(1, -1))
    obs_history = torch.zeros(agent_num,obs_dim*history_leng)
    while True:
        s = time.time()
        while subscribe.poll():
            tmp = subscribe.recv()
            msg = deepcopy(tmp)
            msg_q_trans = T(msg.q); msg_qd_trans = T(msg.qd)
            quat = np.array([msg.quat[1], msg.quat[2], msg.quat[3],msg.quat[0]]).reshape(1, -1)
            lcm_projected_gravity = quat_rotate_inverse(np2torch(quat), np2torch(np.array([[0,0,-1]])))
            obs = torch.cat((lcm_projected_gravity,
                            torch.Tensor([commands]),
                            (np2torch(np.array(msg_q_trans)).reshape(1, -1) - \
                            np2torch(np.array([default_q]))) * 1.0,
                            np2torch(np.array(msg_qd_trans)).reshape(1, -1) * 0.05,
                            action_prev
                            ), dim=-1)
            obs_dict['obs']=obs.clone()
            obs_dict['privileged_obs'] = None
            if init:
                obs_history = torch.cat([obs.clone() for _ in range(history_leng)], dim=-1)
                init = False
            else:
                obs_history = torch.cat((obs_history[:, obs_dim:].clone(), obs.clone()), dim=-1)
            obs_dict['obs_history'] = obs_history.clone()

        if msg is not None:
            with torch.no_grad():
                actions = policy(obs_dict)
            actions_scaled = actions[:, :12] * 0.25
            actions_scaled[:, [0, 3, 6, 9]] *= 0.5  # scale down hip flexion range
            q_des = (actions_scaled + np2torch(np.array([default_q]))).numpy().tolist()
            action_prev = actions.clone()

            q_des_trans = T(q_des[0])
            publish.send(q_des_trans)
        else:
            q_des = np2torch(np.array([default_q])).numpy().tolist()

            q_des_trans = T(q_des[0])
            publish.send(q_des_trans)
        time.sleep(max(interval-(time.time() - s), 0))

if __name__=="__main__":
    device ='cpu' #torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
    recent_runs = sorted(glob.glob(f"{MINI_GYM_ROOT_DIR}/runs/rapid-locomotion/*/*/*"), key=os.path.getmtime)
    logger.configure(Path(recent_runs[-1]).resolve())
    policy = load_policy(device=device)
    subscribe = None
    while True:
        try:
            subscribe = Client('/tmp/lcm_states5')
        except:
            continue
        break
    publish = Listener('/tmp/feedback_output2').accept()

    subscriber_main(policy, subscribe, publish)