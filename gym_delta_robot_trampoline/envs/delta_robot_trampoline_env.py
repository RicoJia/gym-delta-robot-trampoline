import gym
import pybullet as p
import numpy as np
from gym_delta_robot_trampoline.resources.delta_robot_trampoline import Omnid_Simulator
import matplotlib.pyplot as plt
import os
import pybullet_data

"""
Action space (1,3) : [theta_1_torque, theta_2_torque, theta_3_torque]
Observation space (1,18) : [3 joint_positions, 3 joint velocities, 3 eef positions, 3 eef velocities, 3
      3 ball positions, 3 ball velocities]
"""

FAIL_ALTITUDE = 0.20
BONUS_ALTITUDE_DIFF = 0.16
MAX_STEP_NUM = 800

class DeltaRobotTrampolineEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self):
        self.step_counter = 0
        #TODO
        # self.client = p.connect(p.DIRECT)

        self.client = p.connect(p.GUI)
        p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=0, cameraPitch=-40, cameraTargetPosition=[0.05,-0.35,0.2])

        self.action_space = gym.spaces.box.Box(
            low=np.array([-100] * 3),
            high=np.array([100] * 3))

        self.observation_space = gym.spaces.box.Box(
            low=np.array([-np.pi/4, -np.pi/4, -np.pi/4, -100, -100, -100, \
                          -5, -5, -5, -50, -50, -50, \
                          -20, -20, 0, -50, -50, -50]),
            high=np.array([np.pi/2, np.pi/2, np.pi/2, 100, 100, 100, \
                           5, 5, 5, 50, 50, 50, \
                           20, 20, 20, 50, 50, 50]))
        self.np_random, _ = gym.utils.seeding.np_random()

        #enable visualization
        #TODO
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1)

    def reset(self):
        p.resetSimulation()

        # episode params
        self.step_counter = 0
        self.above_BONUS_ALTITUDE_DIFF = False

        p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"))  #loads from the root pybullet library
        p.setGravity(0,0,-10)
        p.setRealTimeSimulation(0)

        #set up the robot and the ball
        self.omnid_simulator = Omnid_Simulator()
        initialized = False
        self.omnid_simulator.attachBallToRobot() # we want the robot to land safely onto the robot.
        while not initialized:
            self.omnid_simulator.updateStates()
            if self.omnid_simulator.ballonRobot():
                self.omnid_simulator.detachBallFromRobot() #now we can let the ball move freely!
                initialized = True
            p.stepSimulation()

        self.observation = self.omnid_simulator.updateStates().astype(np.float32)
        return self.observation

    def step(self, action):
        self.omnid_simulator.applyJointTorque({"theta_1": action[0], \
                                               "theta_2": action[1], \
                                               "theta_3": action[2]})
        p.stepSimulation()
        self.step_counter += 1
        self.observation = self.omnid_simulator.updateStates()

        #z < 0, -100. else, if get over height threshold, we get 100.
        z= self.observation[14]
        if z < FAIL_ALTITUDE:
            reward = -25
            done = True
        else:
            height_diff = z - self.observation[8]
            if height_diff >= BONUS_ALTITUDE_DIFF:
                done = False
                if not self.above_BONUS_ALTITUDE_DIFF:
                    reward = 50
                    self.above_BONUS_ALTITUDE_DIFF = True
                    self.step_counter = 0
                else:
                    reward = 0
            else:   #ball is above the platform but lower than the relative height threshold
                if self.above_BONUS_ALTITUDE_DIFF:
                    self.above_BONUS_ALTITUDE_DIFF = False
                reward = -0.1
                done = False

        if self.step_counter >= MAX_STEP_NUM:
            done = True

        info = {"eef position: ": self.observation[6:9], \
                "ball position: ": self.observation[12:15]}


        return self.observation.astype(np.float32), reward, done, info


    def render(self,  mode='human'):
        """ Render is an interface function. Since we are using GUI, we do not need this.
        We use GUI because computing view matrices and projection matrices is much slower. """
        pass


    def close(self):
        p.disconnect(self.client)

    def seed(self, seed=None):
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        return [seed]
