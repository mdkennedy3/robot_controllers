#!/usr/bin/env python

# Copyright (c) 2015, Fetch Robotics Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Fetch Robotics Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL FETCH ROBOTICS INC. BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
# THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Author: Michael Ferguson
# Autho: Dinesh Thakur

import sys
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from robot_controllers_msgs.msg import QueryControllerStatesAction, \
                                       QueryControllerStatesGoal, \
                                       ControllerState

ACTION_NAME = "/query_controller_states"

ARM_CONTROLLER_NAMES = ['arm_controller/follow_joint_trajectory', 'arm_controller/gravity_compensation', 'arm_controller/weightless_torque',
'arm_controller/velocity', 'arm_controller/torque_control_arm', 'arm_with_torso_controller/follow_joint_trajectory', 'arm_base_controller']

BASE_CONTROLLER_NAMES = ['base_controller', 'base_torque_controller', 'arm_base_controller']

def queryControllerState():

  rospy.loginfo("Connecting to %s..." % ACTION_NAME)
  client = actionlib.SimpleActionClient(ACTION_NAME, QueryControllerStatesAction)
  sucess = client.wait_for_server(rospy.Duration(3))
  if not sucess:
    rospy.logerr("Could not connect to %s" %ACTION_NAME)
    return False, []

  rospy.loginfo("Requesting state of controllers...")

  goal = QueryControllerStatesGoal()
  client.send_goal(goal)
  client.wait_for_result()
  if client.get_state() == GoalStatus.SUCCEEDED:
    result = client.get_result()
    return True, result
  elif client.get_state() == GoalStatus.ABORTED:
    rospy.logerr(client.get_goal_status_text())
    return False, []

def startController(name):
  rospy.loginfo("Connecting to %s..." % ACTION_NAME)
  client = actionlib.SimpleActionClient(ACTION_NAME, QueryControllerStatesAction)
  sucess = client.wait_for_server(rospy.Duration(3))
  if not sucess:
    rospy.logerr("Could not connect to %s" %ACTION_NAME)
    return False

  state = ControllerState()
  state.name = name
  state.state = state.RUNNING

  goal = QueryControllerStatesGoal()
  goal.updates.append(state)

  rospy.loginfo("Requesting that %s be started..." % state.name)
  client.send_goal(goal)
  client.wait_for_result()
  if client.get_state() == GoalStatus.SUCCEEDED:
    rospy.loginfo("Done.")
    return True
  elif client.get_state() == GoalStatus.ABORTED:
    rospy.logerr(client.get_goal_status_text())
    return False

def stopController(name):
  rospy.loginfo("Connecting to %s..." % ACTION_NAME)
  client = actionlib.SimpleActionClient(ACTION_NAME, QueryControllerStatesAction)
  sucess = client.wait_for_server(rospy.Duration(3))
  if not sucess:
    rospy.logerr("Could not connect to %s" %ACTION_NAME)
    return False

  state = ControllerState()
  state.name = name
  state.state = state.STOPPED

  goal = QueryControllerStatesGoal()
  goal.updates.append(state)

  rospy.loginfo("Requesting that %s be stopped..." % state.name)
  client.send_goal(goal)
  client.wait_for_result()
  if client.get_state() == GoalStatus.SUCCEEDED:
    rospy.loginfo("Done.")
    return True
  elif client.get_state() == GoalStatus.ABORTED:
    rospy.logerr(client.get_goal_status_text())
    return False

if __name__ == "__main__":

  if len(sys.argv) < 3:
    print("usage: switch_controller.py <base/arm/both> < <diff,torque>/ <position, velocity, torque> /<torque>> ")
    exit(-1)


  rospy.init_node("switch_robot_controllers")

  name = sys.argv[1]

  if not(name == 'arm' or name == 'base' or name == 'both'):
    print("usage: switch_controller.py <base/arm/both> < <diff,torque>/ <position, velocity, torque> /<torque>> ")
    exit(-1)

  if name == 'base':
    control_type = sys.argv[2]
    if not(control_type == 'diff' or control_type == 'torque'):
      print("usage: switch_controller.py base <diff,torque>")
      exit(-1)

    (success, result) = queryControllerState()

    if success:
      #Stop all running controllers
      rospy.logwarn("WARNING Stopping all base controllers")
      for state in result.state:
        if state.state == state.RUNNING:
          for base_controller_name in BASE_CONTROLLER_NAMES:
            if state.name == base_controller_name:
              print("%s[%s]: is RUNNING" % (state.name, state.type))
              #Stop the controller
              stopController(state.name)

      #Start the requested controller
      if control_type == 'diff':
        startController('base_controller')
      elif control_type == 'torque':
        startController('base_torque_controller')


  if name == 'arm':
    control_type = sys.argv[2]
    if not(control_type == 'position' or control_type == 'velocity' or control_type == 'torque'):
      print("usage: switch_controller.py arm <position, velocity, torque>")
      exit(-1)

    (success, result) = queryControllerState()

    if success:
      #Stop all running controllers
      rospy.logwarn("WARNING Stopping all arm controllers, arm will drop down")
      for state in result.state:
        if state.state == state.RUNNING:
          for arm_controller_name in ARM_CONTROLLER_NAMES:
            if state.name == arm_controller_name:
              print("%s[%s]: is RUNNING" % (state.name, state.type))
              #Stop the controller
              stopController(state.name)

      #Start the requested controller
      if control_type == 'position':
        startController('arm_controller/gravity_compensation')
      elif control_type == 'velocity':
        startController('arm_controller/velocity')
      elif control_type == 'torque':
        startController('arm_controller/torque_control_arm')

  if name == 'both':
    control_type = sys.argv[2]
    if not(control_type == 'torque'):
      print("usage: switch_controller.py both <torque>")
      exit(-1)

    (success, result) = queryControllerState()

    if success:
      #Stop all running controllers
      rospy.logwarn("WARNING Stopping all arm controllers, arm will drop down")
      for state in result.state:
        if state.state == state.RUNNING:
          for base_controller_name in BASE_CONTROLLER_NAMES:
            if state.name == base_controller_name:
              print("%s[%s]: is RUNNING" % (state.name, state.type))
              #Stop the controller
              stopController(state.name)

          for arm_controller_name in ARM_CONTROLLER_NAMES:
            if state.name == arm_controller_name:
              print("%s[%s]: is RUNNING" % (state.name, state.type))
              #Stop the controller
              stopController(state.name)

      #Start the requested controller
      if control_type == 'torque':
        startController('arm_base_controller')
