import torch
import sys 
sys.path.append('..')
from msg.class_lcm import LowLevelCommandPublisher
import time 
from multiprocessing.connection import Client

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
        self.init_joint = [
        -0.807, -1.2, 2.4, \
        0.807, -1.2, 2.4, \
        -0.807, -1.2, 2.4, \
        0.807, -1.2, 2.4, ]
        
        self.q_des      = self.init_joint
        self.default_joint = [-0.1,-0.8,1.62, 
                              0.1,-0.8,1.62, 
                              -0.1, -0.8,1.62, 
                              0.1,-0.8,1.62]
        self.kp_joint = kp_joint
        self.kd_joint = kd_joint 
        # MIT Cheetah:      [FR, FL, RR, RL]
        # Rapid Locomotion: [FL, FR, RL, RR]    
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

def main(subscribe):
    # Hyperparameter 
    HZ = 500 
    interval = 1/HZ
    duartion = 3
    kp_joint = [20.0] * 12
    kd_joint = [0.5] * 12

    lcm_publihser = LowLevelCommandPublisher()
    # Stand up 
    rt = RepeatedTimerInference(interval=interval, lcm_publisher=lcm_publihser, subscribe=subscribe, duration=duartion)
    q_des = rt.q_des

    # Periodic Task 
    while True:
        s = time.time()
        while subscribe.poll():
            tmp = subscribe.recv()
            q_des = tmp.copy()
        lcm_publihser.publisher(q_des=q_des,
                    kp_joint=kp_joint,
                    kd_joint=kd_joint,
                    )
        time.sleep(max(interval-(time.time() - s), 0))

if __name__ == '__main__':
    while True:
        try:
            subscribe_policy_output = Client('/tmp/feedback_output2')
        except:
            continue
        break

    main(subscribe_policy_output)
