import time 
import math
import select
import threading
import lcm 
import math
import numpy as np
from lowlevel_msg.lowlevel_cmd import lowlevel_cmd
from lowlevel_msg.lowlevel_state import lowlevel_state    

def get_rpy_from_quaternion(q):
    w, x, y, z = q
    r = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x ** 2 + y ** 2))
    p = np.arcsin(2 * (w * y - z * x))
    y = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y ** 2 + z ** 2))
    return np.array([r, p, y])

def get_rotation_matrix_from_rpy(rpy):
    """
    Get rotation matrix from the given quaternion.
    Args:
        q (np.array[float[4]]): quaternion [w,x,y,z]
    Returns:
        np.array[float[3,3]]: rotation matrix.
    """
    r, p, y = rpy
    R_x = np.array([[1, 0, 0],
                    [0, math.cos(r), -math.sin(r)],
                    [0, math.sin(r), math.cos(r)]
                    ])

    R_y = np.array([[math.cos(p), 0, math.sin(p)],
                    [0, 1, 0],
                    [-math.sin(p), 0, math.cos(p)]
                    ])

    R_z = np.array([[math.cos(y), -math.sin(y), 0],
                    [math.sin(y), math.cos(y), 0],
                    [0, 0, 1]
                    ])

    rot = np.dot(R_z, np.dot(R_y, R_x))
    return rot

class StateEstimator:
    def __init__(self, lc):
        self.lc = lc 

        self.joint_pos = np.zeros(12)
        self.joint_vel = np.zeros(12) 
        self.tau_est   = np.zeros(12) 
        self.joint_idxs = [3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8]

        self.body_lin_vel = np.zeros(3) 
        self.body_ang_vel = np.zeros(3)
        self.world_lin_vel = np.zeros(3)
        self.world_ang_vel = np.zeros(3)

        self.R = np.eye(3)
        self.euler = np.zeros(3)
        self.body_loc = np.array([0, 0, 0])
        self.body_quat = np.array([0, 0, 0, 1])

        self.vBody = np.zeros(3)
        self.omegaBody = np.zeros(3)
        self.position = np.zeros(3) 
        self.timuprev = time.time()
        # Callback
        self.lowlevel_state_sub = self.lc.subscribe("low_level_states", self._lowlevel_state_callback)
        # init_logger() 
        self.s = time.time()

    def get_dof_pos(self):
        return self.joint_pos[self.joint_idxs]
    
    def get_dof_vel(self):
        return self.joint_vel[self.joint_idxs]
    
    def get_tau_est(self):
        return self.tau_est[self.joint_idxs]
    
    def get_gravity_vector(self):
        grav = np.dot(self.R.T, np.array([0, 0, -1]))
        return grav
    
    def get_lin_vel(self):
        return self.vBody 
    
    def get_ang_vel(self):
        return self.omegaBody

    def get_rpy(self):
        return self.euler

    def get_yaw(self):
        return self.euler[2]

    def get_body_loc(self):
        return self.body_loc

    def get_body_quat(self):
        return self.body_quat

    def _lowlevel_state_callback(self, channel, data):
        msg = lowlevel_state.decode(data)
        self.joint_pos = np.array(msg.q)
        self.joint_vel = np.array(msg.qd)
        self.tau_est   = np.array(msg.tau_est)
        self.euler = np.array(msg.rpy)
        self.R = get_rotation_matrix_from_rpy(self.euler)
        self.vBody     = np.array(msg.vBody)
        self.omegaBody = np.array(msg.omegaBody)
        self.body_loc  = np.array(msg.position)
        self.body_quat = np.array([msg.quat[1], msg.quat[2], msg.quat[3], msg.quat[0]])
    # def _lowlevel_cmd_callback(self, channel, data):
        # msg = lowlevel_cmd.decode(data)
        # print("   q_des    = %s" % str(msg.q_des))
        # print("   qd_des   = %s" % str(msg.qd_des))
        # print("   p_des    = %s" % str(msg.p_des))
        # print("   v_des    = %s" % str(msg.v_des))
        # print("   kp_joint         = %s" % str(msg.kp_joint))
        # print("   kd_joint    = %s" % str(msg.kd_joint))
        # print("   kp_cartesian = %s" % str(msg.kp_cartesian))
        # print("   kd_cartesian      = %s" % str(msg.kd_cartesian))
        # print("   tau_ff       = %s" % str(msg.tau_ff))
        # print("   f_ff  = %s" % str(msg.f_ff))
        # pass 

    def poll(self):
        t = time.time() 
        try: 
            while True:
                timeout = 0.01 
                rfds, wfds, efds = select.select([self.lc.fileno()], [], [], timeout)
                if rfds:
                    # print("message received!")
                    self.lc.handle()
                    # print(f'Freq {1. / (time.time() - t)} Hz'); t = time.time()
                else:
                    continue
        except KeyboardInterrupt:
            pass

    def spin(self):
        self.run_thread = threading.Thread(target=self.poll, daemon=False)
        self.run_thread.start()
        
    def close(self):
        self.lc.unsubscribe(self.lowlevel_state_sub)
if __name__ == "__main__":
    lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=255")
    se = StateEstimator(lc)
    se.poll()
    # se.spin()
