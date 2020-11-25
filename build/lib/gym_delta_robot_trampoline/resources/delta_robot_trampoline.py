#!/usr/bin/env python3
'''
Welcome to the Delta Arm Trampoline Gym!
'''
#Libraries for the URDF path


import sys
sys.path.append(".")
import pybullet as p
import time
import os
import sys, getopt

#Libraries for PyBullet
import pybullet as p
import numpy as np
import pybullet_data
from gym_delta_robot_trampoline.resources.delta_robot_model import Delta_Robot_Model
from gym_delta_robot_trampoline.resources.ball import Ball


class Omnid_Simulator:
  def __init__(self):
      self.omnid_model = Delta_Robot_Model(base_position = [0,0,0.046375])
      self.ball_model = Ball(base_position = [0,0,0.7])

  def attachBallToRobot(self):
      """Need to attach the ball to the robot during initialization, else the ball will fall off"""
      self.ball_model.getSticky()

  def detachBallFromRobot(self):
      """Free the ball so it can be bounced"""
      self.ball_model.getGlossy()

  def updateStates(self):
      """
      Get states of the robot and the ball, including the position and velocity of:
        - each actuated robot joint
        - the robot end effector
        - the ball
      Return: states: [3 joint_positions, 3 joint velocities, 3 eef positions, 3 eef velocities, 3
      3 ball positions, 3 ball velocities]
      """
      self.joint_pos_dict, self.joint_vel_dict = self.omnid_model.getActuatedJointStates()
      self.eef_pos_arr, self.eef_vel_arr = self.omnid_model.getEndEffectorStates()
      self.ball_pos_arr, self.ball_vel_arr = self.ball_model.getBallStates()
      states = np.array(list(self.joint_pos_dict.values()) + list(self.joint_vel_dict.values()) + \
                        self.eef_pos_arr + self.eef_vel_arr \
                           + self.ball_pos_arr + self.ball_vel_arr)
      return states

  def ballonRobot(self):
      if self.ball_pos_arr[2] - self.eef_pos_arr[2] < self.ball_model.radius + 0.01 and np.linalg.norm(self.ball_vel_arr) < 1e-3 :
          return True
      else:
          return False


  def applyJointTorque(self, torqueDict=None):
    self.omnid_model.applyJointTorque(torqueDict)


def modelTestSetup(max_joint_torque):
    debug_tools = []
    for i in range(1,4):
        debug_tools.append(p.addUserDebugParameter('theta_'+str(i)+'_torque', -max_joint_torque , max_joint_torque , 0))
    return debug_tools

def test_robot():
    c = p.connect(p.SHARED_MEMORY)
    if (c < 0):
        c = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.resetSimulation()
    p.loadURDF("plane.urdf" )  #loads from the root pybullet library
    p.setGravity(0, 0, -10.0)
    p.setRealTimeSimulation(0)

    #set up the robot and the ball
    max_joint_torque = 100
    omnid_simulator = Omnid_Simulator()
    timeStep = 0.0005
    p.setTimeStep(timeStep)
    total_step = 0
    initialized = False
    omnid_simulator.attachBallToRobot() # we want the robot to land safely onto the robot.

    #setup user debug tools if needed
    debug_tools = modelTestSetup(max_joint_torque)

    while p.isConnected():
      observation = omnid_simulator.updateStates()
      #TODO

      if not initialized:
          if omnid_simulator.ballonRobot():
              omnid_simulator.detachBallFromRobot() #now we can let the ball move freely!
              initialized = True
              print("initialized")

      torqueDict ={"theta_1": p.readUserDebugParameter(debug_tools[0]), "theta_2": p.readUserDebugParameter(debug_tools[1]), "theta_3": p.readUserDebugParameter(debug_tools[2])}
      omnid_simulator.applyJointTorque(torqueDict)
      total_step += 1
      p.stepSimulation()

if __name__ == "__main__":
    test_robot()
