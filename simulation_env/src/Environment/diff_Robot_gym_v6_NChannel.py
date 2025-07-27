import gymnasium as gym
import numpy as np
from gymnasium import spaces
from Environment.environment_v6_monoChannel import Environment
from gymnasium.spaces import Dict ,Box
from collections import deque

class DiffRobotEnv(gym.Env):
    """Custom Environment that follows gym interface."""

    metadata = {"render_modes": ["human"], "render_fps": 30}

    def __init__(self,Nchannels=4):
        super().__init__()
        self.env=Environment()
        self.Nchannels=Nchannels
        self._max_episode_steps=self.env.maxstep
        self.action_space = spaces.Box(low=np.array([-1.0, -1.0]), high=np.array([1.0, 1.0]), dtype=np.float32)
        self.observation = {"image":[],"scalers":[]}

        self.observation_space =  Dict({"image": Box(low=0,high=255,shape=(self.Nchannels,50,50),dtype=np.uint8), "scalers": Box(low=-1,high=1,shape=(5,))})
        self.temporalStates=deque([np.zeros((1,50,50)).astype(np.uint8) for i in range(self.Nchannels) ],maxlen=self.Nchannels)


    def step(self, action):
        observation, reward, terminated,info=self.env.step(action)
        self.temporalStates.append(np.reshape(observation["image"]*2.55,(1,50,50)).astype(np.uint8))
        self.observation["image"]=np.concatenate(self.temporalStates,axis=0)
        self.observation["scalers"]=observation["scalers"]
        truncated=info["is_failure"] and not info["is_terminal"] 

        return self.observation, reward, terminated, truncated, info

    def reset(self, seed=None, options=None):
        """!_summary_

        @param seed: _description_, defaults to None @type seed: _type_, optional
        @param options: can be None which will return observation and info , 
        NO_INFO : will only return the observation, defaults to None @type options: _type_, optional
        @return: _description_ @rtype: _type_
        """
        self.temporalStates=deque([np.zeros((1,50,50)).astype(np.uint8) for i in range(self.Nchannels)],maxlen=self.Nchannels)
        observation=self.env.reset()
        self.temporalStates.append(np.reshape(observation["image"]*2.55,(1,50,50)).astype(np.uint8))
        self.observation["image"]=np.concatenate(self.temporalStates,axis=0)
        self.observation["scalers"]=observation["scalers"]
        return self.observation, self.env.info


    def render(self):
        pass

    def close(self):
        self.env.reset()
        
        
def print_model_architecture(model, input_tensor):
    def forward_hook(module, input, output):
        print(f"Layer: {module.__class__.__name__}")
        print(f"  Input shape: {str(input[0].shape)}")
        print(f"  Output shape: {str(output.shape)}")
        print("")

    hooks = []
    for layer in model.children():
        hook = layer.register_forward_hook(forward_hook)
        hooks.append(hook)

    try:
        # Run the input tensor through the model
        model(input_tensor)
    finally:
        # Remove the hooks after the forward pass
        for hook in hooks:
            hook.remove()
            
