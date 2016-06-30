/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Fetch Robotics Inc.
 *  Copyright (c) 2013, Unbounded Robotics Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Unbounded Robotics nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Michael Ferguson */

#include <pluginlib/class_list_macros.h>
#include <robot_controllers/weightless_torque.h>

PLUGINLIB_EXPORT_CLASS(robot_controllers::WeightlessTorqueController, robot_controllers::Controller)

namespace robot_controllers
{

int WeightlessTorqueController::init(ros::NodeHandle& nh, ControllerManager* manager)
{
  Controller::init(nh, manager);
  manager_ = manager;

  // Load URDF
  urdf::Model model;
  if (!model.initParam("robot_description"))
  {
    ROS_ERROR("Failed to parse URDF");
    return -1;
  }

  // Load the tree
  KDL::Tree kdl_tree;
  if (!kdl_parser::treeFromUrdfModel(model, kdl_tree))
  {
    ROS_ERROR("Could not construct tree from URDF");
    return -1;
  }

  // Populate the Chain
  std::string root, tip;
  nh.param<std::string>("root", root, "torso_lift_link");
  nh.param<std::string>("tip", tip, "wrist_roll_link");
  if(!kdl_tree.getChain(root, tip, kdl_chain_))
  {
    ROS_ERROR("Could not construct chain from URDF");
    return -1;
  }

  kdl_chain_dynamics_.reset(new KDL::ChainDynParam(kdl_chain_, KDL::Vector(0,0,-9.81)));

  // Init positions
  positions_ = KDL::JntArrayVel(kdl_chain_.getNrOfJoints());
  KDL::SetToZero(positions_.q);
  KDL::SetToZero(positions_.qdot);

  // Init Joint Handles
  joints_.clear();
  for (size_t i = 0; i < kdl_chain_.getNrOfSegments(); ++i)
  {
    if (kdl_chain_.getSegment(i).getJoint().getType() != KDL::Joint::None)
    {
      joint_names_.push_back(kdl_chain_.getSegment(i).getJoint().getName());
    }
    else
    {
      ROS_INFO_STREAM_NAMED("WeightlessTorqueController", "Missing joint " << i);
    }
  }
  size_t num_joints = joint_names_.size();

  for (size_t i = 0; i < num_joints; i++) {
    joints_.push_back(manager_->getJointHandle(joint_names_[i]));
  }

  command_efforts_.clear();
  smoothed_efforts_.clear();
  for (size_t i = 0; i < num_joints; ++i)
  {
    command_efforts_.push_back(0.0);
    smoothed_efforts_.push_back(0.0);
  }
  gravity_compensation_factor_ = 1.0;

  // Subscribe to command
  command_sub_ = nh.subscribe<trajectory_msgs::JointTrajectory>("command", 1,
                    boost::bind(&WeightlessTorqueController::command, this, _1));
  last_command_time_ = ros::Time(0);

  initialized_ = true;

  ROS_INFO_STREAM_NAMED("WeightlessTorqueController", "Inited WeightlessTorqueController " << getName());

  // Should we autostart?
  bool autostart;
  nh.param("autostart", autostart, false);
  if (autostart)
    manager->requestStart(getName());

  return 0;
}

bool WeightlessTorqueController::start()
{
  if (!initialized_)
    return false;
  return true;
}

void WeightlessTorqueController::update(const ros::Time& now, const ros::Duration& dt)
{
  // Need to initialize KDL structs
  if (!initialized_)
    return;

  // Copy command to local to reduce lock contention
  std::vector<double> command_efforts;
  ros::Time last_command_time;
  {
    boost::mutex::scoped_lock lock(mutex_);
    command_efforts = command_efforts_;
    last_command_time = last_command_time_;
  }

  size_t num_joints = joint_names_.size();

  // Get current positions
  for (size_t i = 0; i < num_joints; ++i)
  {
    positions_.q.data[i] = joints_[i]->getPosition();
  }

  // Do the gravity compensation
  KDL::JntArray torques(num_joints);
  kdl_chain_dynamics_->JntToGravity(positions_.q, torques);

  bool command_active = (now - last_command_time) < ros::Duration(0.25);
  if (!command_active)
  {
    for (size_t i = 0; i < num_joints; ++i)
    {
      smoothed_efforts_[i] += (0 - smoothed_efforts_[i]) * dt.toSec()/0.05;
      if (abs(smoothed_efforts_[i]) < 0.001) smoothed_efforts_[i] = 0.0;
    }
    gravity_compensation_factor_ += (1.0 - gravity_compensation_factor_) * dt.toSec()/0.05;
  }
  else
  {
    for (size_t i = 0; i < num_joints; ++i)
    {
      smoothed_efforts_[i] += (command_efforts[i] - smoothed_efforts_[i]) * dt.toSec()/0.05;
    }
  }

  // Update effort command
  for (size_t i = 0; i < num_joints; ++i)
  {
    joints_[i]->setEffort(torques.data[i] * gravity_compensation_factor_ + smoothed_efforts_[i]);
  }
}

void WeightlessTorqueController::command(const trajectory_msgs::JointTrajectory::ConstPtr& goal)
{
  // Need to initialize KDL structs
  if (!initialized_)
  {
    ROS_ERROR("WeightlessTorqueController: Cannot accept goal, controller is not initialized.");
    return;
  }

  if (goal->points.empty())
  {
    // Stop
    ROS_INFO_STREAM_NAMED("WeightlessTorqueController", "WeightlessTorqueController no points");
    manager_->requestStop(getName());
    return;
  }

  size_t num_joints = joints_.size();

  // Find mapping of joint names into message joint names
  std::vector<size_t> mapping(num_joints, std::string::npos);
  for (size_t j = 0; j < num_joints; ++j)
  {
    for (size_t i = 0; i < goal->joint_names.size(); ++i)
    {
      if (joint_names_[j] == goal->joint_names[i])
      {
        mapping[j] = i;
        break;
      }
    }
    if (mapping[j] == std::string::npos)
    {
      ROS_ERROR("Trajectory goal does not match controlled joints");
      return;
    }
  }
  for (size_t i = 0; i < goal->joint_names.size(); ++i)
  {
    if (goal->joint_names[i] == "gravity_compensation")
    {
      gravity_compensation_factor_ = std::max(0.0, std::min(1.5, goal->points[0].effort[i]));
      break;
    }
  }

  std::vector<double> command_efforts(num_joints, 0.0);
  for (size_t j = 0; j < num_joints; ++j)
  {
    if (0) ROS_INFO_STREAM_NAMED("WeightlessTorqueController", "WeightlessTorqueController point " << j << "->" << mapping[j]);

    if (mapping[j] != std::string::npos)
    {
      double effort = goal->points[0].effort[mapping[j]];
      if (!std::isfinite(effort)) effort = 0.0;
      command_efforts[j] = effort;
    }
  }

  ros::Time now(ros::Time::now());
  {
    boost::mutex::scoped_lock lock(mutex_);
    command_efforts_ = command_efforts;
    last_command_time_ = now;
  }

  // Try to start up
  if (manager_->requestStart(getName()) != 0)
  {
    ROS_ERROR("WeightlessTorqueController: Cannot start, blocked by another controller.");
    return;
  }
}


std::vector<std::string> WeightlessTorqueController::getCommandedNames()
{
  std::vector<std::string> names;
  if (initialized_)
  {
    for (size_t i = 0; i < kdl_chain_.getNrOfSegments(); ++i)
      if (kdl_chain_.getSegment(i).getJoint().getType() != KDL::Joint::None)
        names.push_back(kdl_chain_.getSegment(i).getJoint().getName());
  }
  return names;
}

std::vector<std::string> WeightlessTorqueController::getClaimedNames()
{
  // We don't claim anything
  std::vector<std::string> names;
  return names;
}

}  // namespace robot_controllers
