#!/usr/bin/env python
# -*- coding:utf-8 -*-


# Fonte:
# https://github.com/ros-planning/moveit/blob/master/moveit_commander/demos/plan.py


# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. More on these below. We also import `rospy`_ and some messages that we will use:
##

from __future__ import print_function

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
## END_SUB_TUTORIAL


def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


class MoveGroupPythonIntefaceTutorial(object):
    """MoveGroupPythonIntefaceTutorial"""
    def __init__(self):
        super(MoveGroupPythonIntefaceTutorial, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        #rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        gripper_name = "gripper"
        gripper_group = moveit_commander.MoveGroupCommander(gripper_name)
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                        moveit_msgs.msg.DisplayTrajectory,
                                                        queue_size=20)
        planning_frame = move_group.get_planning_frame()
        planning_frame_gripper = gripper_group.get_planning_frame()  
        eef_link = move_group.get_end_effector_link()
        eef_link_gripper = gripper_group.get_end_effector_link()
        group_names = robot.get_group_names()
        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.gripper_group = gripper_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.eef_link_gripper = eef_link_gripper
        self.group_names = group_names


    def go_to_initial_position(self):
        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()
        home_angles =  (0., 0.109, 0.039, -0.203) # joints 1,2,3,4
        joint_goal = home_angles 
        move_group.go(joint_goal, wait=True)
        move_group.stop()
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.02)

    def open_gripper(self):
        move_group = self.gripper_group
        current_position = move_group.get_current_joint_values()
        print("Current gripper position", current_position)
        open_angles =  (0.019,0.019)
        joint_goal = open_angles 
        move_group.go(joint_goal, wait=True)
        move_group.stop()
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.002)

    def close_gripper(self):
        move_group = self.gripper_group
        current_position = move_group.get_current_joint_values()
        current_position = move_group.get_current_joint_values()
        print("Current gripper position", current_position)
        close_angles =  (-0.01, -0.01)
        joint_goal = close_angles 
        move_group.go(joint_goal, wait=True)
        move_group.stop()
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.002)

    def go_to_final_position(self):
        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()
        home_angles =  (1.200, 0.109, 0.039, -0.203) # joints 1,2,3,4
        joint_goal = home_angles 
        move_group.go(joint_goal, wait=True)
        move_group.stop()
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.02)



def main():
  try:
    print("\n----------------------------------------------------------")
    print("\nWelcome to the MoveIt MoveGroup Python Interface Tutorial")
    print("\n----------------------------------------------------------")
    print("\nPress Ctrl-D to exit at any time\n")
    print("\n============ Press `Enter` to begin the tutorial by setting up the moveit_commander ...\n")
    raw_input()
    tutorial = MoveGroupPythonIntefaceTutorial()

    print("\n============ Press `Enter` to close gripper  ...\n")
    raw_input()
    tutorial.go_to_initial_position()

    print("\n============ Press `Enter` to open gripper ...\n")
    raw_input()
    tutorial.open_gripper()

    print("\n============ Press `Enter` to close gripper  ...\n")
    raw_input()
    tutorial.close_gripper()

    print("\n============ Press `Enter` to close gripper  ...\n")
    raw_input()
    tutorial.go_to_final_position()

 


  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()