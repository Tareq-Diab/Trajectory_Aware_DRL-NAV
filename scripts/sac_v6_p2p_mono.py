import gymnasium as gym
import numpy as np
from stable_baselines3 import SAC
from Environment.diff_Robot_gym_v6_monoChannel_p2p import DiffRobotEnv 
import sys
from TrainingCallback import EpisodeRewardLoggingCallback
from datetime import datetime
import random
from stable_baselines3.common.utils import set_random_seed
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--experiment", default="")               
args = parser.parse_args()
print(f"running {args.experiment}")


sys.setrecursionlimit(10000) 
np.random.seed(42)
random.seed(42)
set_random_seed(42)
env = DiffRobotEnv(nactors=10)
logDir="traing_logs"
runName="sac_mono_channel-{}-{}".format(args.experiment,datetime.now())

runName="sac-p2p-mono-{}-{}".format(args.experiment,datetime.now())
callback=EpisodeRewardLoggingCallback(env,logDir,verbose=1,evalfrequency=10,run_name=runName)

# The noise objects for TD3
n_actions = env.action_space.shape[-1]
print(env.observation_space)

model = SAC("MultiInputPolicy", env, verbose=1,tensorboard_log="{}/{}".format(logDir,runName),seed=42)
vec_env = model.get_env()
callback=EpisodeRewardLoggingCallback(vec_env,logDir,verbose=1,evalfrequency=10,run_name=runName)
for param_tensor in model.actor.state_dict():
    print(param_tensor, "\t", model.actor.state_dict()[param_tensor].size())
print("-----------------------")
print(model.actor)
model.learn(total_timesteps=350_000 , progress_bar=True,log_interval=20,callback=callback)
model.save("{}/{}/sac_diffrobot_{}".format(logDir,runName,runName))

