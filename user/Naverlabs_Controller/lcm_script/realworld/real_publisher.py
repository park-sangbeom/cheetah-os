import sys
sys.path.append('..')
from lowlevel_msg.class_lcm import LCMSubscriber
from multiprocessing.connection import Listener
import time

def main():
    # Socket Listener to publish msg 
    socket_publisher = Listener('/tmp/lcm_states').accept()
    # LCM Subscriber to get lcm_states from a robot 
    lcm_subscriber = LCMSubscriber(VERBOSE=False)
    last_send = time.time()
    while True:
        lcm_subscriber.lc.handle()
        if time.time() - last_send > 0.02:
            socket_publisher.send(lcm_subscriber.msg)
            last_send = time.time()
if __name__=="__main__":
    main()