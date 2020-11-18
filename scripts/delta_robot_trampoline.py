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


class Omnid_Simulator:
  def __init__(self, urdf_path):
      urdf_name = "delta_robot_pybullet.urdf"
      self.omnid_model = Delta_Robot_Model(urdfRootPath=urdf_path,
                                         urdf_name = urdf_name,
                                         base_position = [0,0,0.046375]    #we have -0.046375 as our base offset, due to the setup in URDF
                                         )

  # def jointStatesPub(self):
  #   self.joint_state_pub.publish(self.omnid_model.returnJointStateMsg())
  # def executeMotionCommands(self):
  #   self.omnid_model.executeAllMotorPosCommands()
  # def updateJointStates_CB(self, msg):
  #   """
  #   API function for as a ROS Subscriber callback_function updating joint positions.
  #   """
  #   self.omnid_model.updateJointStates(msg.name, msg.position)
  #
  # def processFollowTrajectory(self, goal):
  #     """
  #     Call back function for Moveit! trajectory following action request.
  #     :param goal: the goal trajectory of actuated joints, i.e, phi angles.
  #     """
  #     success = True
  #     traj = goal.trajectory
  #     num_points = len(traj.points)
  #     # figure out joint names and their positions
  #     joint_names = traj.joint_names
  #     #start from point 1, since the first point is the current starting point
  #     for i in range(1, num_points):
  #         #check for pre-emption
  #         if self.action_server.is_preempt_requested():
  #             rospy.loginfo("%Trajectory Action Preempted on Omnid" )
  #             self.action_server.set_preempted()
  #             success = False
  #             break
  #         #figure out the duration and joint positions of each trajectory segment
  #         duration = (traj.points[i].time_from_start - traj.points[i-1].time_from_start ).to_sec()
  #         joint_positions = traj.points[i].positions
  #         #update the joints
  #         self.omnid_model.updateJointStates(joint_names, joint_positions)
  #         #realize each segment and time it
  #         r = rospy.Rate(1.0/duration)
  #         r.sleep()
  #         #check if the action has been preempted
  #
  #     if success:
  #         msg = 'Trajectory completed'
  #         rospy.loginfo(msg)
  #         res = FollowJointTrajectoryResult()
  #         self.action_server.set_succeeded(result=res, text=msg)


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
    while p.isConnected():
    #   omnid_simulator.executeMotionCommands()
    #   omnid_simulator.jointStatesPub()
      p.stepSimulation()
