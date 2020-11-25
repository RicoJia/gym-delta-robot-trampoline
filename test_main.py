import gym
import torch
from agent import Agent
import gym_delta_robot_trampoline
import time

def main():
    nn = torch.nn.Sequential(torch.nn.Linear(8, 64), torch.nn.Tanh(),
                             torch.nn.Linear(64, 2))
    # agent = TRPOAgent(policy=nn)
    #
    # agent.load_model("agent.pth")
    # agent.train("SimpleDriving-v0", seed=0, batch_size=5000, iterations=100,
    #             max_episode_length=250, verbose=True)
    # agent.save_model("agent.pth")
    env = gym.make('delta_robot_trampoline-v0')
    ob = env.reset()
    while True:
        action = [0,0,0]
        ob, _, done, _ = env.step(action)
        env.render()
        if done:
            ob = env.reset()
            time.sleep(1/30)


if __name__ == '__main__':
    main()
