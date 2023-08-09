import sys
sys.path.append('..')
from lowlevel_cmd_publisher import LowLevelCommandPublisher
import time 
import argparse 
# MIT Cheetah:              [FR, FL, RR, RL]
# Rapid Locomotion (Isaac): [FL, FR, RL, RR]  
class RepeatedTimerInference(object):
    def __init__(self, interval, lcm_publisher, subscribe=None, duration=2, 
                 kp_joint=[20.0] * 12, kd_joint=[0.5] * 12):
        self.interval   = interval
        self.lcm_publisher = lcm_publisher
        self.duration   = duration
        self.subscribe  = subscribe
        
        self.down_pose = [-0.7890825271606445, -0.9639183282852173, 2.725369453430176, 
                           0.7814531326293945, -0.9799400568008423, 2.779815435409546, 
                           -0.9088659286499023, -0.9816077947616577, 2.6333565711975098, 
                           0.8165483474731445, -1.0071662664413452, 2.6456193923950195]
        
        self.up_pose = [-0.1,-0.8,1.62, 
                              0.1,-0.8,1.62, 
                              -0.1, -0.8,1.62, 
                              0.1,-0.8,1.62]

        self.kp_joint = kp_joint
        self.kd_joint = kd_joint 
  
    def start(self, pose):
        self.step_time  = 0
        if pose=='down': 
            while self.step_time < self.duration:
                s = time.time()
                self.stand_down()
                time.sleep(max(self.interval-(time.time() - s), 0))

        elif pose=='up':
            while self.step_time < self.duration:
                s = time.time()
                self.stand_up()
                time.sleep(max(self.interval-(time.time() - s), 0))

    def stand_up(self):
        self.q_des = [self.down_pose[i] + (self.up_pose[i] - self.down_pose[i]) * (self.step_time/self.duration) for i in range(len(self.down_pose))]
        self.lcm_publisher.publisher(q_des=self.q_des,
                    kp_joint=self.kp_joint,
                    kd_joint=self.kd_joint)
        self.step_time +=self.interval

    def stand_down(self):
        self.q_des = [self.up_pose[i] - (self.up_pose[i] - self.down_pose[i]) * (self.step_time/self.duration) for i in range(len(self.down_pose))]
        self.lcm_publisher.publisher(q_des=self.q_des,
                    kp_joint=self.kp_joint,
                    kd_joint=self.kd_joint)
        self.step_time +=self.interval

def stand_up_publisher():
    # Hyperparameter 
    HZ = 50 
    interval = 1/HZ
    duartion = 3
    lcm_publihser = LowLevelCommandPublisher()
    # Stand up 
    rt = RepeatedTimerInference(interval=interval, lcm_publisher=lcm_publihser, duration=duartion)
    rt.start(pose='up')

