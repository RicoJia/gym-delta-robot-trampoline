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
from delta_robot_trampoline.resources.delta_robot_model import Delta_Robot_Model
from delta_robot_trampoline.resources.ball import Ball


class Omnid_Simulator:
  def __init__(self):
      self.omnid_model = Delta_Robot_Model(
                                         urdf_name = "delta_robot_pybullet.urdf",
                                         base_position = [0,0,0.046375]    #we have -0.046375 as our base offset, due to the setup in URDF
                                         )

      self.ball_model = Ball(
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

def checkCommandLine(argv):
    test_model = None
    try:
        opts, args = getopt.getopt(argv,"ht",["help", "test"])
    except getopt.GetoptError:
        print ('Please see -h or --help for help')
        sys.exit(2)
    for opt, arg in opts:
        if opt in ("-h", "--help"):
            print ('-t or --test is for testing the PyBullet model with a slider bar. If the input arg is empty, we are going to launch the environment')
            sys.exit(1)
        elif opt in ("-t", "--test"):
            test_model = True
        else:
            test_model = False
    return test_model



def modelTestSetup(test_model, max_joint_torque):
    if not test_model:
        return None
    else:
        debug_tools = []
        for i in range(1,4):
            debug_tools.append(p.addUserDebugParameter('theta_'+str(i)+'_torque', -max_joint_torque , max_joint_torque , 0))
        return debug_tools

def test_robot():
    #TODO
    # test_model = checkCommandLine(sys.argv[1:])
    test_model = True
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
    debug_tools = modelTestSetup(test_model, max_joint_torque)

    while p.isConnected():
      omnid_simulator.updateStates()
      if not initialized:
          if omnid_simulator.ballonRobot():
              omnid_simulator.detachBallFromRobot() #now we can let the ball move freely!
              initialized = True
              print("initialized")

      torqueDict = {"theta_1": 0, "theta_2": 0, "theta_3": 0} if not test_model else \
          {"theta_1": p.readUserDebugParameter(debug_tools[0]), "theta_2": p.readUserDebugParameter(debug_tools[1]), "theta_3": p.readUserDebugParameter(debug_tools[2])}
      omnid_simulator.applyJointTorque(torqueDict)
      total_step += 1
      p.stepSimulation()

if __name__ == "__main__":
    test_robot()
