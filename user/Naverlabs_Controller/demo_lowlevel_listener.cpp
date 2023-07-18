#include <stdio.h>
#include <lcm/lcm-cpp.hpp>

#include "lowlevel_cmd.hpp"
#include "lowlevel_state.hpp"

class Handler {
    public:
        ~Handler() {}
        void handleMessage(const lcm::ReceiveBuffer *rbuf, const std::string &chan, 
                           const lowlevel_cmd *msg)
    {
        (void)rbuf;
        int i;
        printf("Received message on channel \"%s\":\n", chan.c_str());
        printf("  q_des    = (%f, %f, %f)\n", msg->q_des[0], msg->q_des[1],
               msg->q_des[2]);
        printf("  qd_des = (%f, %f, %f, %f)\n", msg->qd_des[0], msg->qd_des[1],
               msg->qd_des[2], msg->qd_des[3]);
        printf("  kp_joint:");
        for (i = 0; i < 12; i++)
            printf(" %f", msg->kp_joint[i]);
        printf("\n");
    }
};

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    lcm::LCM lcm;

    if (!lcm.good())
        return 1;

    Handler handlerObject;
    lcm.subscribe("low_level_cmds", &Handler::handleMessage, &handlerObject);

    while (0 == lcm.handle()) {
        // Do nothing
    }

    return 0;
}
