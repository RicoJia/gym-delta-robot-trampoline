#!/usr/bin/env python3
'''
The Delta Arm Model
'''

import pybullet as p
import numpy as np
import os

class Ball:
    def __init__(self, urdf_name, base_position = [0,0,0]):
        self.reset(urdf_name, base_position)

    def reset(self,urdf_name, base_position):
        urdf_name = os.path.join(os.path.dirname(__file__), urdf_name)
        self.model_unique_id = p.loadURDF(urdf_name, basePosition=base_position, globalScaling=0.1)
        p.changeVisualShape(self.model_unique_id,-1,rgbaColor=[0.8,0.8,0.8,1])
        self.buildParamLists()

    def buildParamLists(self):
        self.radius = 0.5

    def getSticky(self):
        p.changeDynamics(self.model_unique_id ,-1,linearDamping=10, angularDamping=100000, rollingFriction=100000, spinningFriction=100000)

    def getGlossy(self):
        p.changeDynamics(self.model_unique_id ,-1,linearDamping=0, angularDamping=0, rollingFriction=0.001, spinningFriction=0.001)

    def getBallStates(self):
        link_state_vel = p.getBaseVelocity(bodyUniqueId = self.model_unique_id)
        link_state_pos = p.getBasePositionAndOrientation(bodyUniqueId = self.model_unique_id)
        link_pos = link_state_pos[0]
        link_vel = link_state_vel[0]
        return (link_pos, link_vel)


