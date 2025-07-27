import gymnasium as gym
import numpy as np
from   gymnasium import spaces
from Environment.environment_v2_p2p import Environment as env_p2p
from Environment.environment_v2_raw_p2p_actors import Environment as env_p2p_actors

class DiffRobotEnv(gym.Env):
    """Custom Environment that follows gym interface."""

    metadata = {"render_modes": ["human"], "render_fps": 30}

    def __init__(self,old=False,mode="actors",nactors=12):
        super().__init__()
        self.env=env_p2p() if mode =="simple" else  env_p2p_actors(number_actors=nactors)
        self.old=old
        self._max_episode_steps=self.env.maxstep
        self.action_space = spaces.Box(low=np.array([-1.0, -1.0]), high=np.array([1.0, 1.0]), dtype=np.float32)
        self.observation_space = spaces.Box(low=-1, high=1,shape=(self.env.lase_res+5,), dtype=np.float32)

    def step(self, action):
        observation, reward, terminated,info=self.env.step(action)
        observation= np.array(observation,dtype=np.float32)
        truncated=info["is_failure"] and not info["is_terminal"]  
        if self.old: 
            return observation, reward, terminated, info
        return observation, reward, terminated, truncated, info

    def reset(self, seed=None, options=None):
        """!_summary_

        @param seed: _description_, defaults to None @type seed: _type_, optional
        @param options: can be None which will return observation and info , 
        NO_INFO : will only return the observation, defaults to None @type options: _type_, optional
        @return: _description_ @rtype: _type_
        """
        if options=="NO_INFO" or self.old:
            return self.env.reset()

        return self.env.reset(), self.env.info


    def render(self):
        pass

    def close(self):
        self.env.reset()
        