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

#Libraries for PyBullet
import pybullet as p
import numpy as np
import pybullet_data
from delta_robot_model import Delta_Robot_Model
from ball import Ball


class Omnid_Simulator:
  def __init__(self, urdf_path):
      self.omnid_model = Delta_Robot_Model(urdfRootPath=urdf_path,
                                         urdf_name = "delta_robot_pybullet.urdf",
                                         base_position = [0,0,0.046375]    #we have -0.046375 as our base offset, due to the setup in URDF
                                         )

      self.ball_model = Ball(urdfRootPath=urdf_path,
                             urdf_name = "soccerball.urdf",
                             base_position = [0,0,0.8]
                             )

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
      """
      self.joint_pos_dict, self.joint_vel_dict = self.omnid_model.getActuatedJointStates()
      self.eef_pos_arr, self.eef_vel_arr = self.omnid_model.getEndEffectorStates()
      self.ball_pos_arr, self.ball_vel_arr = self.ball_model.getBallStates()


  def ballonRobot(self):
      if self.ball_pos_arr[2] - self.eef_pos_arr[2] < self.ball_model.radius + 0.01 and np.linalg.norm(self.ball_vel_arr) < 1e-4 :
          return True
      else:
          return False


  def applyJointTorque(self, torqueDict=None):
    self.omnid_model.applyJointTorque(torqueDict)


if __name__ == "__main__":
    c = p.connect(p.SHARED_MEMORY)
    if (c < 0):
        c = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.resetSimulation()
    p.loadURDF("plane.urdf" )  #loads from the root pybullet library
    p.setGravity(0, 0, -10.0)
    urdf_path = os.path.join(os.getcwd(),'../urdf')
    print(urdf_path)
    omnid_simulator = Omnid_Simulator(urdf_path)
    #
    p.setRealTimeSimulation(0)

    global joint_values
    timeStep = 0.0005
    p.setTimeStep(timeStep)
    total_step = 0
    initialized = False
    omnid_simulator.attachBallToRobot() # we want the robot to land safely onto the robot.
    while p.isConnected():
      omnid_simulator.updateStates()
      if not initialized:
          if omnid_simulator.ballonRobot():
              omnid_simulator.detachBallFromRobot() #now we can let the ball move freely!
              initialized = True
              print("initialized")
    #
      torqueDict = {"theta_1": 0, "theta_2": 0, "theta_3": 0}
      omnid_simulator.applyJointTorque(torqueDict)
      total_step += 1
      p.stepSimulation()
