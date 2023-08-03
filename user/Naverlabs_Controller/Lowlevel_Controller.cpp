#include "Lowlevel_Controller.hpp"
void Lowlevel_Controller::initializeController(){

    _legController->_maxTorque = userParameters.max_tau;
    _legController->_legsEnabled = true;
    _lcm.subscribe("low_level_cmds",&Lowlevel_Controller::handleLowlevelCmdLCM,this);

}

void Lowlevel_Controller::runController() {

    ++l_iter;

    if (l_iter % 5000 == 0 && l_iter < userParameters.max_iter)
        printf("[LowLevel_Ctrl] INFO: Control iteration is %ld now ...\n\n", l_iter);

    if (userParameters.calibrate <= 0.4 && !_legController->_zeroEncoders) {
//      Main Loop
        if (l_iter >= 100 && l_iter <= userParameters.max_iter) {
            pub_sub_lcm();
        }else{
            all_Passive();
        }
    }
}

/*
 *  Unable all motors to stop the robot
 * */
void Lowlevel_Controller::all_Passive(){

    for (int leg = 0; leg < 4; ++leg) {
        _legController->commands[leg].kpJoint.setZero();
        _legController->commands[leg].kdJoint.setZero();
        _legController->commands[leg].kpCartesian.setZero();
        _legController->commands[leg].kdCartesian.setZero();
        for (int jidx = 0; jidx < 3; ++jidx) {
            _legController->commands[leg].tauFeedForward[jidx] = 0.;
        }
    }
}
/*
 * Handle commands from Python
 * */
void Lowlevel_Controller::handleLowlevelCmdLCM(const lcm::ReceiveBuffer *rbuf,
                                             const std::string &chan,
                                             const lowlevel_cmd *msg) {
    (void)rbuf;
    (void)chan;

    // _mtx.lock();
    memcpy(&_cmd, msg, sizeof(lowlevel_cmd));
    // _mtx.unlock();    
    printf("[LowLevel Ctrl] Received LCM message\n");
    //    Subscribe Commands
}
/*
 * Subs commands from Python Controller and send it to robot
 * Publish states to Python Controller
 * */
void Lowlevel_Controller::pub_sub_lcm() {
    lowlevel_state low_state_lcmt;
//    Publish States
    memset(&low_state_lcmt, 0, sizeof(low_state_lcmt));
    auto& seResult = _stateEstimator->getResult();
    for (int i = 0; i < 3; ++i) {
        low_state_lcmt.rpy[i] = seResult.rpy[i];
        low_state_lcmt.position[i] = seResult.position[i];
        low_state_lcmt.vBody[i] = seResult.vBody[i];
        low_state_lcmt.omegaBody[i] = seResult.omegaBody[i];
        low_state_lcmt.vWorld[i] = seResult.vWorld[i];
        low_state_lcmt.omegaWorld[i] = seResult.omegaWorld[i];
        low_state_lcmt.aBody[i] = seResult.aBody[i];
        low_state_lcmt.aWorld[i] = seResult.aWorld[i];
    }
    for (int leg = 0; leg < 4; ++leg) {
        low_state_lcmt.quat[leg] = seResult.orientation[leg];
        for (int i = 0; i < 3; ++i) {
            low_state_lcmt.tau_est[leg*3+i] = _legController->datas[leg].tauEstimate[i];
            low_state_lcmt.q[leg*3+i] = _legController->datas[leg].q[i];
            low_state_lcmt.qd[leg*3+i] = _legController->datas[leg].qd[i];
            low_state_lcmt.p[leg*3+i] = _legController->datas[leg].p[i];
            low_state_lcmt.v[leg*3+i] = _legController->datas[leg].v[i];
        }
    }
    _lcm.publish("low_level_states",&low_state_lcmt);

//    Send Commands to Robot
    {
    _mtx.lock();
        for (int leg = 0; leg < 4; ++leg) {
            for (int i = 0; i < 3; ++i) {
                _legController->commands[leg].forceFeedForward[i] = _cmd.f_ff[3*leg+i];
                _legController->commands[leg].tauFeedForward[i] = _cmd.tau_ff[3*leg+i];
                _legController->commands[leg].qDes[i] = _cmd.q_des[3*leg+i];
                _legController->commands[leg].qdDes[i] = _cmd.qd_des[3*leg+i];
                _legController->commands[leg].kpJoint(i,i) = _cmd.kp_joint[3*leg+i];
                _legController->commands[leg].kdJoint(i,i) = _cmd.kd_joint[3*leg+i];
                _legController->commands[leg].pDes[i] = _cmd.p_des[3*leg+i];
                _legController->commands[leg].vDes[i] = _cmd.v_des[3*leg+i];
                _legController->commands[leg].kpCartesian(i,i) = _cmd.kp_cartesian[3*leg+i];
                _legController->commands[leg].kdCartesian(i,i) = _cmd.kd_cartesian[3*leg+i];
                if (_legController->commands[leg].tauFeedForward[i] > userParameters.max_tau){
                    printf("[LowLevel Ctrl] Torque has exceeded the max torque limitation, modify %f to %f",
                        _legController->commands[leg].tauFeedForward[i],userParameters.max_tau);
                    _legController->commands[leg].tauFeedForward[i] = userParameters.max_tau;
                }
            // printf("[Leg: %d][qDes: %f] \n", 3*leg+i, _legController->commands[leg].qDes[i]);
            }
        }
    _mtx.unlock();
    }

}
