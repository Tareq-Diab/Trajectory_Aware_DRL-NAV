import gymnasium as gym
import numpy as np
from stable_baselines3 import SAC
from Environment.diff_Robot_gym_v6_NChannel import DiffRobotEnv 
import sys
from TrainingCallback import EpisodeRewardLoggingCallback
from datetime import datetime
import random
from stable_baselines3.common.utils import set_random_seed

sys.setrecursionlimit(10000) 
np.random.seed(42)
random.seed(42)
set_random_seed(42)

sys.setrecursionlimit(10000) 
env = DiffRobotEnv()
logDir="traing_logs"
runName="sac_stacked-states-{}".format(datetime.now())
callback=EpisodeRewardLoggingCallback(env,logDir,verbose=1,evalfrequency=10,run_name=runName)


model = SAC("MultiInputPolicy", env, verbose=1,tensorboard_log="{}/{}".format(logDir,runName),seed=42)
vec_env = model.get_env()
callback=EpisodeRewardLoggingCallback(vec_env,logDir,verbose=1,evalfrequency=10,run_name=runName)
for param_tensor in model.actor.state_dict():
    print(param_tensor, "\t", model.actor.state_dict()[param_tensor].size())
print("-----------------------")
print(model.actor)
model.learn(total_timesteps=350_000 , progress_bar=True,log_interval=20,callback=callback)
model.save("{}/{}/sac_stacked_repeat_{}".format(logDir,runName,runName))


