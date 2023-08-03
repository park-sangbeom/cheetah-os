from msg.class_lcm import LowLevelStateSubscriber
from multiprocessing.connection import Client, Listener
import time

def publisher_main():
    publish = Listener('/tmp/lcm_states5').accept()
    lcm_module = LowLevelStateSubscriber()
    last_send = time.time()
    while True:
        s= time.time()
        lcm_module.lc.handle()
        if time.time() - last_send > 0.02:
            publish.send(lcm_module.msg)
            last_send = time.time()
if __name__=="__main__":
    publisher_main()