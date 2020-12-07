import gym
import torch
import numpy as np
import gym_delta_robot_trampoline

import utils
import os
from DDPG import DDPG
import time

# Runs policy for X episodes and returns average reward
# A fixed seed is used for the eval environment
def eval_policy(policy, eval_env, seed, eval_episodes = 3, test=False):
    eval_env.seed(seed + 100)
    avg_reward = 0
    for i in range(eval_episodes):
        state, done = eval_env.reset(), False
        while not done:
            action = policy.select_action(np.array(state))
            state, reward, done, _ = eval_env.step(action)
            avg_reward += reward
            if test:
                time.sleep(1.0/240.0)   #this is the default PyBullet Update Frequency.
    avg_reward /= eval_episodes
    print("---------------------------------------")
    print(f"Evaluation over {eval_episodes} episodes: {avg_reward:.3f}")
    print("---------------------------------------")
    return avg_reward

def is_non_zero_file(fpath):
    return os.path.isfile(fpath) and os.path.getsize(fpath) > 0

def main():
    nn = torch.nn.Sequential(torch.nn.Linear(8, 64), torch.nn.Tanh(),
                             torch.nn.Linear(64, 2))
    test = True
    state_dim = 18
    action_dim = 3
    max_action = 20
    batch_size = 20
    cov_scale = 0.3
    eval_freq = 10
    episode_reward = 0
    episode_timesteps = 0
    episode_num = 0
    random_exploration_ep = 100

    env_name = 'delta_robot_trampoline-v0'

    env = gym.make(env_name)
    env.seed(0)
    torch.manual_seed(0)
    np.random.seed(0)
    policy = DDPG(state_dim, action_dim, max_action)

    policy.load(f"./models/model")

    replay_buffer = utils.ReplayBuffer(state_dim, action_dim)

    # Evaluate untrained policy
    if not test:
        if is_non_zero_file(os.getcwd()+"/results/evaluations.npy"):
            evaluations = list(np.load(os.getcwd()+"/results/evaluations.npy")) + [eval_policy(policy, env, 0)]
        else:
            evaluations = [eval_policy(policy, env, 0)]
        evaluated = True

    state, done = env.reset(), False

    while True:
            if not test:
                episode_timesteps += 1
                #TODO
                # Select action randomly or according to policy
                if episode_num < random_exploration_ep:
                # if (episode_timesteps+1)%random_exploration_ep == 0:
                    action = env.action_space.sample()
                else:
                    action = (
                        policy.select_action(np.array(state))
                        + np.random.normal(0, max_action * cov_scale, size=action_dim)
                    ).clip(-max_action, max_action)

                # Perform action
                next_state, reward, done, _ = env.step(action)
                done_bool = float(done)

                # Store data in replay buffer
                replay_buffer.add(state, action, next_state, reward, done_bool)
                state = next_state  #why update your states right here?
                episode_reward += reward


                #TODO
                # Train agent after collecting sufficient data
                if episode_num >= random_exploration_ep:
                    policy.train(replay_buffer, batch_size)

                if done:
                    # +1 to account for 0 indexing. +0 on ep_timesteps since it will increment +1 even if done=True
                    print(f" Episode Num: {episode_num+1} Episode steps: {episode_timesteps} Reward: {episode_reward:.3f}")
                    # Reset environment
                    state, done = env.reset(), False
                    episode_reward = 0
                    episode_timesteps = 0
                    episode_num += 1

                # Evaluate episode
                if episode_num % eval_freq == 0 and not evaluated:
                    evaluations.append(eval_policy(policy, env, 1))
                    np.save(f"./results/evaluations", evaluations)
                    policy.save(f"./models/model")
                    evaluated = True
                elif episode_num % eval_freq != 0:
                    evaluated = False
            else:
                policy.load(f"./models/model")
                eval_policy(policy, env, 1, test=True)



if __name__ == '__main__':
    main()
