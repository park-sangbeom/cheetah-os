import torch
import sys 
sys.path.append('..')
from msg.class_lcm import LowLevelCommandPublisher
import time 
from multiprocessing.connection import Client
import numpy as np 
import json 

def np2torch(x_np,device='cpu'):
    if x_np is None:
        x_torch = None
    else:
        x_torch = torch.tensor(x_np,dtype=torch.float32,device=device)
    return x_torch

class RepeatedTimerInference(object):
    def __init__(self, interval, lcm_publisher, subscribe, duration, 
                 kp_joint=[20.0] * 12, kd_joint=[0.5] * 12):
        self.interval   = interval
        self.lcm_publisher = lcm_publisher
        self.step_time  = 0
        self.duration   = duration
        self.subscribe  = subscribe
        self.init_joint = [-0.7890825271606445, -0.9639183282852173, 2.725369453430176, 
                           0.7814531326293945, -0.9799400568008423, 2.779815435409546, 
                           -0.9088659286499023, -0.9816077947616577, 2.6333565711975098, 
                           0.8165483474731445, -1.0071662664413452, 2.6456193923950195]

        self.q_des      = self.init_joint
        self.default_joint = [-0.1,-0.8,1.62, 
                              0.1,-0.8,1.62, 
                              -0.1, -0.8,1.62, 
                              0.1,-0.8,1.62]
        self.kp_joint = kp_joint
        self.kd_joint = kd_joint    
        self.start()

    def _run(self):
        self.stand_up()

    def start(self):
        while self.step_time < self.duration:
            s = time.time()
            self._run()
            time.sleep(max(self.interval-(time.time() - s), 0))

    def stand_up(self):
        while self.subscribe.poll():
            self.subscribe.recv()
        self.q_des = [self.init_joint[i] + (self.default_joint[i] - self.init_joint[i]) * (self.step_time/self.duration) for i in range(len(self.init_joint))]
        self.lcm_publisher.publisher(q_des=self.q_des,
                    kp_joint=self.kp_joint,
                    kd_joint=self.kd_joint)
        self.step_time +=self.interval

def inference_main(subscribed_policy_socket):
    limit_low_joint = np.array([-1., -2., -2., -1., -2., -2., -1., -2., -2., -1., -2., -2.])
    limit_high_joint = np.array([1., 2., 2., 1., 2., 2., 1., 2., 2., 1., 2., 2.])
    # Hyperparameter 
    HZ = 500 
    interval = 1/HZ
    duartion = 3
    kp_joint = [20.0] * 12
    kd_joint = [0.5] * 12
    # Low Level Command Publisher
    lcm_publisher = LowLevelCommandPublisher()
    # Stand up 
    rt = RepeatedTimerInference(interval=interval, lcm_publisher=lcm_publisher, subscribe=subscribed_policy_socket, duration=duartion)
    q_des = rt.q_des
    real_time = time.time()
    # Periodic Task 
    while (time.time()-real_time)<8.0:
        s = time.time()
        while subscribed_policy_socket.poll():
            tmp = subscribed_policy_socket.recv()
            q_des = tmp.copy()
            # Limit Joint 
            q_des = np.array(q_des)
            limit_high_idx = np.where(q_des<limit_low_joint)
            q_des[limit_high_idx] = limit_low_joint[limit_high_idx]
            limit_low_idx = np.where(q_des>limit_high_joint)
            q_des[limit_low_idx] = limit_high_joint[limit_low_idx]
        lcm_publisher.publisher(q_des=q_des,
                    kp_joint=kp_joint,
                    kd_joint=kd_joint,
                    )
        time.sleep(max(interval-(time.time() - s), 0))
    # Trot gait 
    trot_motion=[]; trot_joint_cnt = 0
    for line in open('trot_joint.json','r'):
        data = json.loads(line)
        trot_motion.append([data['q'][0]+data['q'][1]+data['q'][2]+data['q'][3]])
    while trot_joint_cnt<len(trot_motion[:400]):
        s = time.time()
        lcm_publisher.publisher(q_des=trot_motion[trot_joint_cnt][0],
                    kp_joint=kp_joint,
                    kd_joint=kd_joint)
        trot_joint_cnt +=1 #+=interval
        time.sleep(max(interval-(time.time() - s), 0))
    print("Done.")

if __name__ == '__main__':
    while True:
        try:
            subscribed_policy_socket = Client('/tmp/feedback_output2')
        except:
            continue
        break

    inference_main(subscribed_policy_socket)
