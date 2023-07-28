
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

def T(qin):
    tmp = list(qin).copy()
    qout = tmp[3:6] + tmp[0:3] + tmp[9:12] + tmp[6:9] 
    assert len(qout) == 12
    return qout


def subscriber_main(contrl_dt, socket_subscriber, socket_publisher):
    msg = None 
    q_des = [
        -0.807, -1.2, 1.2, \
        0.807, -1.2, 2.4, \
        -0.807, -1.2, 2.4, \
        0.807, -1.2, 2.4, ]
    while True:
        s = time.time()
        while socket_subscriber.poll():
            tmp = socket_subscriber.recv()
            msg = deepcopy(tmp)
        if msg is not None:
            socket_publisher.send(q_des)
        else: 
            socket_publisher.send(q_des)

        time.sleep(max(contrl_dt-(time.time() - s), 0))
        # time.sleep(max(0.02-(time.time() - s), 0))

if __name__=="__main__":
    socket_subscriber = None
    while True:
        try:
            # Socket Client to get lcm_states from another thread 
            socket_subscriber = Client('/tmp/lcm_states')
        except:
            continue
        break
    # Socket Listener to publish the action from a model 
    socket_publisher = Listener('/tmp/feedback_output2').accept()
    contrl_dt = 0.002 
    subscriber_main(contrl_dt, socket_subscriber, socket_publisher)