/*********************************************************************
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Fetch Robotics Inc.
 *  Copyright (c) 2013, Unbounded Robotics Inc.
 *  Copyright (c) 2008, Willow Garage, Inc.
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

/*
 * Derived a bit from pr2_controllers/cartesian_pose_controller.cpp
 * Author: Michael Ferguson, Wim Meeussen
 */

#include <pluginlib/class_list_macros.h>
#include <robot_controllers/velocity.h>

#include <urdf/model.h>

PLUGINLIB_EXPORT_CLASS(robot_controllers::VelocityController, robot_controllers::Controller)

namespace robot_controllers
{

VelocityController::VelocityController() :
  initialized_(false)
{
}

int VelocityController::init(ros::NodeHandle& nh, ControllerManager* manager)
{
  // We absolutely need access to the controller manager
  if (!manager)
  {
    initialized_ = false;
    return -1;
  }

  Controller::init(nh, manager);
  manager_ = manager;

  // Get Joint Names
  joint_names_.clear();
  XmlRpc::XmlRpcValue names;
  if (!nh.getParam("joints", names))
  {
    ROS_ERROR_STREAM("No joints given for " << nh.getNamespace());
    return -1;
  }
  if (names.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR_STREAM("Joints not in a list for " << nh.getNamespace());
    return -1;
  }
  for (int i = 0; i < names.size(); ++i)
  {
    XmlRpc::XmlRpcValue &name_value = names[i];
    if (name_value.getType() != XmlRpc::XmlRpcValue::TypeString)
    {
      ROS_ERROR_STREAM("Not all joint names are strings for " << nh.getNamespace());
      return -1;
    }
    joint_names_.push_back(static_cast<std::string>(name_value));
    ROS_INFO_STREAM_NAMED("VelocityController", "Joint " << i << " " << joint_names_.back());

  }
  size_t num_joints = joint_names_.size();


  // Get Joint Handles, setup feedback
  joints_.clear();
  //feedback_.joint_names.clear();
  for (size_t i = 0; i < num_joints; ++i)
  {
    JointHandlePtr j = manager_->getJointHandle(joint_names_[i]);
    //feedback_.joint_names.push_back(j->getName());
    joints_.push_back(j);
    continuous_.push_back(j->isContinuous());
  }

  command_efforts_.clear();
  smoothed_efforts_.clear();
  for (size_t i = 0; i < num_joints; ++i)
  {
    command_efforts_.push_back(0.0);
    smoothed_efforts_.push_back(0.0);
  }

  // Subscribe to command
  command_sub_ = nh.subscribe<trajectory_msgs::JointTrajectory>("command", 1,
                    boost::bind(&VelocityController::command, this, _1));
  last_command_time_ = ros::Time(0);

  initialized_ = true;

  ROS_INFO_STREAM_NAMED("VelocityController", "Inited VelocityController " << getName());

  return 0;
}

bool VelocityController::start()
{
  if (!initialized_)
  {
    ROS_ERROR_NAMED("VelocityController",
                    "Unable to start, not initialized.");
    return false;
  }

  return true;
}

bool VelocityController::stop(bool force)
{
  return true;
}

bool VelocityController::reset()
{
  // Simply stop
  return (manager_->requestStop(getName()) == 0);
}

void VelocityController::update(const ros::Time& now, const ros::Duration& dt)
{
  if (!initialized_)
    return;  // Should never really hit this

  // Copy command to local to reduce lock contention
  std::vector<double> command_efforts;
  ros::Time last_command_time;
  {
    boost::mutex::scoped_lock lock(mutex_);
    command_efforts = command_efforts_;
    last_command_time = last_command_time_;
  }

  size_t num_joints = joints_.size();

  if (0) ROS_INFO_STREAM_NAMED("VelocityController", "VelocityController update " << now << " " << last_command_time);

  if ((now - last_command_time) > ros::Duration(0.5))
  {
    manager_->requestStop(getName());
  }
  else if ((now - last_command_time) > ros::Duration(0.1))
  {
    for (size_t i = 0; i < num_joints; ++i)
    {
      smoothed_efforts_[i] += (0 - smoothed_efforts_[i]) * dt.toSec()/0.05;
    }
  }
  else
  {
    for (size_t i = 0; i < num_joints; ++i)
    {
      smoothed_efforts_[i] += (command_efforts[i] - smoothed_efforts_[i]) * dt.toSec()/0.05;
    }
  }
  if (num_joints == 7) {
    if (1) ROS_INFO_STREAM_NAMED("VelocityController", "VelocityController setVelocity " <<
      smoothed_efforts_[0] << " " <<
      smoothed_efforts_[1] << " " <<
      smoothed_efforts_[2] << " " <<
      smoothed_efforts_[3] << " " <<
      smoothed_efforts_[4] << " " <<
      smoothed_efforts_[5] << " " <<
      smoothed_efforts_[6]);
  }
  for (size_t i = 0; i < num_joints; ++i)
  {
    joints_[i]->setVelocity(smoothed_efforts_[i], 0.0);
  }
}

void VelocityController::command(const trajectory_msgs::JointTrajectory::ConstPtr& goal)
{
  // Need to initialize KDL structs
  if (!initialized_)
  {
    ROS_ERROR("VelocityController: Cannot accept goal, controller is not initialized.");
    return;
  }

  if (goal->points.empty())
  {
    // Stop
    ROS_INFO_STREAM_NAMED("VelocityController", "VelocityController no points");
    manager_->requestStop(getName());
    return;
  }

  size_t num_joints = joints_.size();

  if (goal->joint_names.size() != num_joints)
  {
    ROS_ERROR("Trajectory goal size does not match controlled joints size.");
    return;
  }
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

  std::vector<double> command_efforts(num_joints, 0.0);
  for (size_t j = 0; j < num_joints; ++j)
  {
    if (0) ROS_INFO_STREAM_NAMED("VelocityController", "VelocityController point " << j << "->" << mapping[j]);

    if (mapping[j] != std::string::npos) {
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
    ROS_ERROR("VelocityController: Cannot start, blocked by another controller.");
    return;
  }
}

std::vector<std::string> VelocityController::getCommandedNames()
{
  return joint_names_;
}

std::vector<std::string> VelocityController::getClaimedNames()
{
  return getCommandedNames();
}

}  // namespace robot_controllers
