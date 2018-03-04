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

// Author: Dinesh Thakur, allow individual joint velocity control.

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

  command_velocities_.clear();
  smoothed_velocities_.clear();
  for (size_t i = 0; i < num_joints; ++i)
  {
    command_velocities_.push_back(0.0);
    smoothed_velocities_.push_back(0.0);
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
  std::vector<double> command_velocities;
  ros::Time last_command_time;
  {
    boost::mutex::scoped_lock lock(mutex_);
    command_velocities = command_velocities_;
    last_command_time = last_command_time_;
  }

  size_t num_joints = joints_.size();

  if (0) ROS_INFO_STREAM_NAMED("VelocityController", "VelocityController update " << now << " " << last_command_time);

  bool command_active = (now - last_command_time) < ros::Duration(0.5);

  if (!command_active)
  {
    manager_->requestStop(getName());
  }
  else
  {
    for (size_t i = 0; i < num_joints; ++i)
    {
      if(command_velocities[i] != std::numeric_limits<double>::max())
        joints_[i]->setVelocity(command_velocities[i], 0.0); //TODO limit velocities to max/min
    }
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

  /*
  if (goal->joint_names.size() != num_joints)
  {
    ROS_ERROR("Trajectory goal size does not match controlled joints size.");
    return;
  }
  */

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
    /*
    if (mapping[j] == std::string::npos)
    {
      ROS_WARN("Trajectory goal does not match controlled joints");
      return;
    }*/
  }

  std::vector<double> command_velocities(num_joints, std::numeric_limits<double>::max());
  for (size_t j = 0; j < num_joints; ++j)
  {
    if (0) ROS_INFO_STREAM_NAMED("VelocityController", "VelocityController point " << j << "->" << mapping[j]);

    if (mapping[j] != std::string::npos) {
      double velocities = goal->points[0].velocities[mapping[j]];
      if (!std::isfinite(velocities)) velocities = 0.0;
      command_velocities[j] = velocities;
    }
  }

  ros::Time now(ros::Time::now());
  {
    boost::mutex::scoped_lock lock(mutex_);
    command_velocities_ = command_velocities;
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
