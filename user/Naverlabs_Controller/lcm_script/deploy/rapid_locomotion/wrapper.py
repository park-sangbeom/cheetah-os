import torch 

class Wrapper: 
    def __init__(self, env, obs_history_length):
        self.env = env 
        self.obs_history_length = obs_history_length    
        self.num_obs_history = self.obs_history_length * 1
        self.obs_history = torch.zeros(1, self.num_obs_history, dtype=torch.float,
                                       device=self.env.device, requires_grad=False)
        
    def step(self, action):
        obs = self.env.step(action)
        self.obs_history = torch.cat((self.obs_history[:, 42:], obs), dim=1)
        return {"obs": obs, "privileged_obs":None, "obs_history": self.obs_history}

    def get_observations(self):
        obs =self.env.get_obs()
        self.obs_history = torch.cat((self.obs_history[:, 42:], obs), dim=1)
        return {"obs": obs, "privileged_obs":None, "obs_history": self.obs_history}    

    def reset(self):
        obs = self.env.reset()
        self.obs_history[:,:]=0
        return {"obs":obs, "privileged_obs":None, "obs_history":self.obs_history}
    
    