from msg.class_lcm import LowLevelStateSubscriber
from multiprocessing.connection import Client, Listener
import time

def main():
    publish = Listener('/tmp/lcm_states').accept()

    # lcm_module = LightweightCommunicationsMarshalling(VERBOSE=False)
    lcm_module = LowLevelStateSubscriber()
    s=0
    last_send = time.time()
    while True:
        if time.time() - s > 0.025:
            print(time.time() - s)
        s= time.time()
        # lcm_module.subscriber()
        lcm_module.lc.handle()
        if time.time() - last_send > 0.02:
            publish.send(lcm_module.msg)
            last_send = time.time()
if __name__=="__main__":
    main()