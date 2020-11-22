import gym
import pybullet as p
import numpy as np
from gym_delta_robot_trampoline.resources.delta_robot_trampoline import Omnid_Simulator
import matplotlib.pyplot as plt
import os
import pybullet_data

"""
Action (torque) array: [theta_1, theta_2, theta_3]
Observation (1,18) array: [3 joint_positions, 3 joint velocities, 3 eef positions, 3 eef velocities, 3
      3 ball positions, 3 ball velocities]
"""

class DeltaRobotTrampolineEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self):
        self.step_counter = 0
        #TODO
        # self.client = p.connect(p.DIRECT)

        self.client = p.connect(p.GUI)
        p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=0, cameraPitch=-40, cameraTargetPosition=[0.55,-0.35,0.2])

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
        # self.reset()
        # trivial tracking variables
        self.rendered_img = None
        self.step_limit = 2000
        self.height_thre = 0.15
        self.above_height_thre = False
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1)

    def reset(self):
        p.resetSimulation()
        #TODO
        # p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0) # we will enable rendering after we loaded everything

        self.step_counter = 0
        p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"))  #loads from the root pybullet library
        # p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0) # we will enable rendering after we loaded everything
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
        print("Env has been reset")
        self.observation = self.omnid_simulator.updateStates().astype(np.float32)
        #TODO
        # p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1)
        return self.observation

    def step(self, action):
        # p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING)
        self.omnid_simulator.applyJointTorque({"theta_1": action[0], \
                                               "theta_2": action[1], \
                                               "theta_3": action[2]})
        p.stepSimulation()
        self.step_counter += 1
        print(self.step_counter)
        self.observation = self.omnid_simulator.updateStates()

        #z < 0, -100. else, if get over height threshold, we get 100.
        z= self.observation[14]
        if z < 0.10:
            reward = -100
            done = True
        elif z >= self.height_thre:
            done = False
            if not self.above_height_thre:
                reward = 10
                self.above_height_thre = True
                self.step_counter = 0
            else:
                reward = 0
        else:
            if self.above_height_thre:
                self.above_height_thre = False
            reward = 0
            done = False

        if self.step_counter == self.step_limit:
            done = True

        info = {"eef position: ": self.observation[6:9], \
                "ball position: ": self.observation[12:15]}

        return self.observation.astype(np.float32), reward, done, info


    def render(self,  mode='human'):
        pass
        #This is pretty slow
        # if self.rendered_img is None:
        #     self.rendered_img = plt.imshow(np.zeros((720, 960, 4)))
        # view_matrix = p.computeViewMatrixFromYawPitchRoll(cameraTargetPosition=[0.7,0,0.05],
        #                                                   distance=2.0,
        #                                                   yaw=90,
        #                                                   pitch=-10,
        #                                                   roll=0,
        #                                                   upAxisIndex=2)
        # proj_matrix = p.computeProjectionMatrixFOV(fov=60,
        #                                              aspect=float(960) /720,
        #                                              nearVal=0.1,
        #                                              farVal=100.0)
        # (_, _, px, _, _) = p.getCameraImage(width=960,
        #                                       height=720,
        #                                       viewMatrix=view_matrix,
        #                                       projectionMatrix=proj_matrix,
        #                                       renderer=p.ER_BULLET_HARDWARE_OPENGL)
        #
        # rgb_array = np.array(px, dtype=np.uint8)
        # rgb_array = np.reshape(rgb_array, (720,960, 4))
        # self.rendered_img.set_data(rgb_array)
        # if plt.fignum_exists(1):
        #     plt.draw()
            # plt.pause(.0001)




    def close(self):
        p.disconnect(self.client)

    def seed(self, seed=None):
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        return [seed]
