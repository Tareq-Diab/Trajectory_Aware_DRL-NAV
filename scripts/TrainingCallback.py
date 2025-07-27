import numpy as np
from stable_baselines3.common.callbacks import BaseCallback
import time
import os
class EpisodeRewardLoggingCallback(BaseCallback):
    def __init__(self,eval_env, log_dir: str, verbose=1,evalfrequency=10,run_name=""):
        super(EpisodeRewardLoggingCallback, self).__init__(verbose)
        os.makedirs(log_dir+"/"+run_name, exist_ok=True)
        self.log_dir = log_dir
        self.episode_rewards = []
        self.episode_lengths = []
        self.eval_rewards=[]
        self.episode_timestamp=[]
        self.eval_env=eval_env
        self.evalfrequency=evalfrequency
        self.start_time=time.time()
        self.run_name=run_name
        
    def _on_step(self) -> bool:
        if 'episode' in self.locals['infos'][0]:
            episode_reward = self.locals['infos'][0]['episode']['r']
            episode_length = self.locals['infos'][0]['episode']['l']
            self.episode_rewards.append(episode_reward)
            self.episode_lengths.append(episode_length)
            self.episode_timestamp.append(time.time()-self.start_time)
            # if self.verbose > 0:
                # print(f"Episode reward: {episode_reward}, Episode length: {episode_length}")
            if (len(self.episode_rewards) % self.evalfrequency) == 0 :
                evalScore=self.evaluate_agent()
                # print("at episode {} the agent eval is {}".format( len(self.episode_rewards) , evalScore) )
                self.eval_rewards.append([evalScore,np.sum(self.episode_lengths) ,time.time()-self.start_time])
        return True
    
    def evaluate_agent(self):
        rewards = []
        for _ in range(2):
            done = False
            state = self.eval_env.reset()
            total_reward = 0.0
            while not done:
                action, _ = self.model.predict(state)
                state, reward, done, _ = self.eval_env.step(action)
                total_reward += reward
            rewards.append(total_reward)
        mean_reward = np.mean(rewards)
        return mean_reward

    def _on_training_end(self) -> None:
        # Save logged rewards and episode lengths
        np.save(self.log_dir + '/{}/episode_rewards.npy'.format(self.run_name), np.array(self.episode_rewards))
        np.save(self.log_dir + '/{}/episode_lengths.npy'.format(self.run_name), np.array(self.episode_lengths))
        np.save(self.log_dir + '/{}/episode_timestamp.npy'.format(self.run_name), np.array(self.episode_timestamp))
        np.save(self.log_dir + '/{}/episode_evals.npy'.format(self.run_name),np.array(self.eval_rewards))
        
