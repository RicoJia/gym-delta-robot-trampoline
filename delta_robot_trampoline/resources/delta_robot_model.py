#!/usr/bin/env python3
'''
The Delta Arm Model
'''
import os
import pybullet as p
import numpy as np

class Delta_Robot_Model:

  def __init__(self, urdf_name, base_position = [0,0,0]):
    self.reset(urdf_name, base_position)

  def reset(self,urdf_name, base_position):
    self.buildParamLists()
    urdf_name = os.path.join(os.path.dirname(__file__), urdf_name)
    self.model_unique_id = p.loadURDF(urdf_name, basePosition=base_position, useFixedBase=True)
    self.buildLookups()
    self.resetLinkFrictions(lateral_friction_coefficient=0)
    self.resetJointsAndMotors()
    self.buildClosedChains()

  def buildParamLists(self):
      self.leg_num = 3
      self.kp = 1
      self.kd = 0.6
      self.max_motor_force = 150000

      self.end_effector_thickness = 0.01 #thickness of the end effector platform
      self.end_effector_radius = 0.062
      self.upper_leg_names = {1: "upper_leg_1", 2: "upper_leg_2", 3: "upper_leg_3"}
      self.upper_leg_length = 0.368
      self.leg_pos_on_end_effector = {"upper_leg_1": 0.0, "upper_leg_2": 2.0*np.pi/3.0, "upper_leg_3": 4.0*np.pi/3.0} #angular positions of upper legs on the platform
      self.after_spring_joint_vals = {"theta_1": 0.0, "theta_2": 0.0, "theta_3": 0.0} #name of the after_spring_joint: joint_initial_value

      self.end_effector_name = "platform_link"
      self.end_effector_home_vals = {"x": 0.0, "y": 0.0, "z": 0.095}

      # joints to be initialized
      self.init_joint_values = {**self.after_spring_joint_vals, **self.end_effector_home_vals}
      self.motorDict = {**self.after_spring_joint_vals}

  def buildLookups(self):
    """
    Build following look ups in dictionaries:
     1. jointNameToId
     2. linkNameToID
    Note that since each link and its parent joint has the same ID. Base frame by link_id = -1.
    A very important assumption here is in your URDF, you first have a world link, then have a base_link
    """
    # jointNameToId, linkNameToID
    nJoints = p.getNumJoints(self.model_unique_id)
    self.jointNameToId = {}
    self.linkNameToID={}
    for i in range(nJoints):
      jointInfo = p.getJointInfo(self.model_unique_id, i)
      self.jointNameToId[jointInfo[1].decode('UTF-8')] = jointInfo[0]
      self.linkNameToID[jointInfo[12].decode('UTF-8')] = jointInfo[0]

  def resetLinkFrictions(self, lateral_friction_coefficient):
      """
      Reset all links friction
      """
      for id in self.linkNameToID.values():
          p.changeDynamics(bodyUniqueId=self.model_unique_id,
                           linkIndex=id,
                           jointLowerLimit = -1000,
                           jointUpperLimit = 1000,
                           jointLimitForce = 0,
                           lateralFriction=lateral_friction_coefficient,
                           spinningFriction=0.0,
                           rollingFriction=0.0,
                           linearDamping = 0.0,
                           angularDamping = 0.0,
                           jointDamping = 0.0,
                           contactStiffness=0.0,
                           contactDamping=0.0,
                           maxJointVelocity=10000
                           )

  def resetJointsAndMotors(self):
    """
    We do two things here:
      1. Look up the URDF and set the desired joint angles.
      2. Set up motor control: 1. Specify which joints need to be controlled by motors. 2. specify motor control parameters
    """
    self.maxPoint2PointForce = 5000000

    #disable friction in all joints
    for joint_id in self.jointNameToId.values():
        p.setJointMotorControl2(self.model_unique_id, joint_id,
                                controlMode=p.VELOCITY_CONTROL, force=0)

    #All joint values to be initialized
    for joint_name in self.init_joint_values:
      p.resetJointState(bodyUniqueId = self.model_unique_id, jointIndex=self.jointNameToId[joint_name], targetValue=self.init_joint_values[joint_name])


  def buildClosedChains(self):
    """
    Connect links to joints to build closed chain robots, since URDF does not support chain robots.
    """
    joint_axis = [0,0,0]
    for leg_id in range(1, self.leg_num + 1):
        upper_leg_name = self.upper_leg_names[leg_id]
        x = self.end_effector_radius * np.cos(self.leg_pos_on_end_effector[upper_leg_name])
        y = self.end_effector_radius * np.sin(self.leg_pos_on_end_effector[upper_leg_name])
        parent_frame_pos = np.array([ x, y, -self.end_effector_thickness/2.0])  # Cartesian coordnates on the platform, r_platform = 0.062
        child_frame_pos = [self.upper_leg_length, 0.0, 0.0]   # L_upper = 0.368/2.0
        new_joint_id = p.createConstraint(self.model_unique_id, self.linkNameToID[self.end_effector_name],
                                          self.model_unique_id, self.linkNameToID[upper_leg_name],
                                          p.JOINT_POINT2POINT, joint_axis, parent_frame_pos, child_frame_pos)
        p.changeConstraint(new_joint_id, maxForce=self.maxPoint2PointForce)


# ########################## Helpers ##########################
  def setMotorValueByName(self, motorName, desiredValue):
    """
    Joint Position Control using PyBullet's Default joint angle control
    :param motorName: string, motor name
    :param desiredValue: float, angle value
    """
    motorId=self.jointNameToId[motorName]
    p.setJointMotorControl2(bodyIndex=self.model_unique_id,
                          jointIndex=motorId,
                          controlMode=p.POSITION_CONTROL,
                          targetPosition=desiredValue,
                          positionGain=self.kp,
                          velocityGain=self.kd,
                          force=self.max_motor_force)

  def applyJointTorque(self, torqueDict):
      """
      This is reserved for training.
      """
      after_spring_joint_ids = [self.jointNameToId[name] for name in torqueDict]
      p.setJointMotorControlArray(
                      bodyIndex=self.model_unique_id,
                      jointIndices=after_spring_joint_ids,
                      controlMode=p.TORQUE_CONTROL,
                      forces= torqueDict.values()
      )

  def getActuatedJointStates(self):
      joint_pos = {}
      joint_vel = {}
      for joint_name in self.motorDict.keys():
          joint_state = p.getJointState(bodyUniqueId = self.model_unique_id, jointIndex=self.jointNameToId[joint_name])
          joint_pos[joint_name] = joint_state[0]
          joint_vel[joint_name] = joint_state[1]
      return (joint_pos, joint_vel)

  def getEndEffectorStates(self):
      link_state = p.getLinkState(bodyUniqueId = self.model_unique_id, linkIndex = self.linkNameToID[self.end_effector_name], computeLinkVelocity=1)
      link_pos = link_state[0]
      link_vel = link_state[6]
      return (link_pos, link_vel)

  # def executeAllMotorPosCommands(self):
  #   """
  #   This is a helper function that executes all motor position commands in self.motorDict
  #   """
  #   for name, value in self.motorDict.items():
  #     self.setMotorValueByName(name, value)

#   def returnJointStateMsg(self):
#       """
#       Publish joint states. Note that effort is the motor torque applied during the last stepSimulation.
#       :return:
#       """
#       joint_state_msg = JointState()
#       for joint_name in self.jointNameToId.keys():
#           joint_state = p.getJointState(bodyUniqueId = self.model_unique_id, jointIndex=self.jointNameToId[joint_name])
#           joint_state_msg.name.append(joint_name)
#           joint_state_msg.position.append(joint_state[0])
#           joint_state_msg.velocity.append(joint_state[1])
#           joint_state_msg.effort.append(joint_state[3])
#       return joint_state_msg
#
#   def updateJointStates(self, name_ls, position_ls):
#       """
#       The core function to update joint states.  If a joint name is not for an actuated joint, it will be skipped without notice.
#       :param name_ls: (list-like) names of joints to be updated.
#       :param position_ls: (list-like) new positions of joints
#       """
#       for i in range(len(name_ls)):
#           if name_ls[i] in self.motorDict:
#               self.motorDict[name_ls[i]] = position_ls[i]




