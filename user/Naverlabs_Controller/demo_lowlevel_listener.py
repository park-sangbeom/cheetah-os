# Wang Yinuo, 07/25/2021, dbdxwyn@163.com
# If you want to control robot by python, use this example and add your controller to it.
import lcm
from exlcm import lowlevel_cmd, lowlevel_state
import sys 
import json 
# Receive robot state from C++
def robot_state_handler(channel, data):
    msg = lowlevel_state.decode(data)
    lcm.EventLog("data.txt", mode='w',overwrite=True).write_event(1, channel,data)
    # print("Received message on channel \"%s\"" % channel)
    # print("   q    = %s" % str(msg.q))
    # print("   qd   = %s" % str(msg.qd))
    # print("   p    = %s" % str(msg.p))
    # print("   v    = %s" % str(msg.v))
    # print("   rpy         = %s" % str(msg.rpy))
    # print("   position    = %s" % str(msg.position))
    # print("   orientation = %s" % str(msg.quat))
    # print("   vWorld      = %s" % str(msg.vWorld))
    # print("   vBody       = %s" % str(msg.vBody))
    # print("   omegaWorld  = %s" % str(msg.omegaWorld))
    # print("   omegaBody   = %s" % str(msg.omegaBody))
    # print("   aWorld      = %s" % str(msg.aWorld))
    # print("   aBody       = %s" % str(msg.aBody))
    # print(" ")

# LCM communication
lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=255")
lc.subscribe("low_level_states", robot_state_handler)
try:
    while True:
        lc.handle()
        log = lcm.EventLog('data.txt', "r")
        for event in log:
            if event.channel == "low_level_states":
                msg = lowlevel_state.decode(event.data)
                print("   q    = %s" % str(msg.q))
                print("   qd   = %s" % str(msg.qd))
                print("   p    = %s" % str(msg.p))
                print("   v    = %s" % str(msg.v))
                print("   rpy         = %s" % str(msg.rpy))
                print("   position    = %s" % str(msg.position))
                print("   orientation = %s" % str(msg.quat))
                print("   vWorld      = %s" % str(msg.vWorld))
                print("   vBody       = %s" % str(msg.vBody))
                print("   omegaWorld  = %s" % str(msg.omegaWorld))
                print("   omegaBody   = %s" % str(msg.omegaBody))
                print("   aWorld      = %s" % str(msg.aWorld))
                print("   aBody       = %s" % str(msg.aBody))
                print(" ")

except KeyboardInterrupt:
    pass
