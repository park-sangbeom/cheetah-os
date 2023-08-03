import sys
sys.path.append('..')
from msg.class_lcm import LowLevelCommandPublisher
import time 

class RepeatedTimerInference(object):
    def __init__(self, interval, lcm_publisher, subscribe=None, duration=2, 
                 kp_joint=[20.0] * 12, kd_joint=[0.5] * 12):
        self.interval   = interval
        self.lcm_publisher = lcm_publisher
        self.step_time  = 0
        self.duration   = duration
        self.subscribe  = subscribe
        # self.init_joint = [
        # -0.807, -1.2, 2.4, \
        # 0.807, -1.2, 2.4, \
        # -0.807, -1.2, 2.4, \
        # 0.807, -1.2, 2.4, ]

        self.init_joint = [-0.7890825271606445, -0.9639183282852173, 2.725369453430176, 
                           0.7814531326293945, -0.9799400568008423, 2.779815435409546, 
                           -0.9088659286499023, -0.9816077947616577, 2.6333565711975098, 
                           0.8165483474731445, -1.0071662664413452, 2.6456193923950195]

        self.q_des      = self.init_joint
        self.default_joint = self.init_joint
        # self.default_joint = [-0.1,-0.8,1.62, 
        #                       0.1,-0.8,1.62, 
        #                       -0.1, -0.8,1.62, 
        #                       0.1,-0.8,1.62]
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

        while self.step_time < self.duration+3: 
            s = time.time()
            self._run()
            time.sleep(max(self.interval-(time.time() - s), 0))


    def stand_up(self):
        if self.step_time<self.duration:
            self.q_des = [self.init_joint[i] + (self.default_joint[i] - self.init_joint[i]) * (self.step_time/self.duration) for i in range(len(self.init_joint))]
        else: 
            self.q_des = self.default_joint
        self.lcm_publisher.publisher(q_des=self.q_des,
                    kp_joint=self.kp_joint,
                    kd_joint=self.kd_joint)
        self.step_time +=self.interval

def main():
    # Hyperparameter 
    HZ = 500 
    interval = 1/HZ
    duartion = 3
    kp_joint = [20.0] * 12
    kd_joint = [0.5] * 12

    lcm_publihser = LowLevelCommandPublisher()
    # Stand up 
    rt = RepeatedTimerInference(interval=interval, lcm_publisher=lcm_publihser, duration=duartion)

if __name__ == '__main__':
    s = time.time()
    main()
    print(time.time()-s)
